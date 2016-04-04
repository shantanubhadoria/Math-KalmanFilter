use strict;
use warnings;
package Math::KalmanFilter;

# PODNAME: Math::KalmanFilter
# ABSTRACT: Kalman Filter(also known as Linear Quadratic Estimation) implementation for sensor fusion and such
# COPYRIGHT
# VERSION

# Dependencies
use 5.010;
use Moose;

=attr qAngle

 * default: 0.001 

=cut

has qAngle => (
    is      => 'rw',
    default => 0.001,
);

=attr qBias

 * default: 0.003

=cut

has qBias => (
    is      => 'rw',
    default => 0.003,
);

=attr rMeasure 

 * default: 0.03

=cut 

has rMeasure => (
    is      => 'rw',
    default => 0.03,
);

=attr bias

 * starting value(default): 0
 * recalculated(optimised) at each new sensor reading.

=cut

has bias => (
    is      => 'rw',
    default => 0,
);

=attr covariance

This is the covariance matrix, it is stored as a 2d array ref
 * starting value(default): [[0,0],[0,0]]
 * recalculated(optimised) at each new sensor reading.

=cut

has covariance => (
    is      =>'rw',
    default => sub {
        return [
            [0,0],
            [0,0]
        ];
    }
);

=attr angle 

Calculated angle

=cut

has angle => (
    is      =>'rw',
);

=method getAngle

Calculate new state based on observed reading from state sensor, delta sensor and time elapsed since last reading.

=cut

sub getAngle {
    my ($self, $newAngle, $newRate, $deltaTime) = @_;
    
    my $covariance = $self->covariance;
    my $angle      = $self->angle;
    my $bias       = $self->bias;

    my $rate = $newAngle - $bias;
    $angle += $deltaTime * $rate;

    $covariance->[0]->[0] += $deltaTime * ( $deltaTime * $covariance->[1]->[1] - $covariance->[0]->[1] - $covariance->[1]->[0] + $self->qAngle);
    $covariance->[0]->[1] -= $deltaTime * $covariance->[1]->[1];
    $covariance->[1]->[0] -= $deltaTime * $covariance->[1]->[1];
    $covariance->[1]->[1] += $deltaTime * $self->qBias;

    my $innovationCovariance = $covariance->[0]->[0] + $self->rMeasure;

    my $kalmanGain = [
        $covariance->[0]->[0] / $innovationCovariance,
        $covariance->[1]->[0] / $innovationCovariance,
        ];

    my $y = $newAngle - $angle;
    $angle += $kalmanGain->[0] * $y;
    $bias  += $kalmanGain->[1] * $y;

    $covariance->[0]->[0] -= $kalmanGain->[0] * $covariance->[0]->[0];
    $covariance->[0]->[1] -= $kalmanGain->[0] * $covariance->[0]->[1];
    $covariance->[1]->[0] -= $kalmanGain->[1] * $covariance->[0]->[0];
    $covariance->[1]->[1] -= $kalmanGain->[1] * $covariance->[0]->[1];

    $self->covariance($covariance);
    $self->angle($angle);

    return $angle;
}

1;

__END__

=for HTML <a href="https://travis-ci.org/shantanubhadoria/perl-Math-KalmanFilter"><img src="https://travis-ci.org/shantanubhadoria/perl-Math-KalmanFilter.svg?branch=master"></a>

=begin wikidoc

= SYNOPSIS

    use Math::KalmanFilter;
    use Time::HiRes qw(time);

    my $oldTime = time();
    # Read State from state sensor, in a IMU this would be one of the accelerometer orientation angle 
    # e.g. Angle between orientation vector and X axis in degrees.
    my $state = readStateSensor(); 

    # Read rate of change of state, in a IMU gyroscope measures the delta i.e. the Rate of change of
    # $state e.g. rate of change of angle between orientation vector and X axis in degree per second.
    my $delta = readDeltaSensor(); 

    #Created a Kalman filter object to hold state changes for your measurement target.
    $kalman = Math::KalmanFilter->new(
        angle => $state
    );

    while($keep_running){
        my $newTime = time();
        my $deltaTime = $newTime - $oldTime;
        $oldTime  = $newTime;

        my $state = readStateSensor(); 
        my $delta = readDeltaSensor(); 
        my $angle = $kalman->getAngle($state,$delta,$deltaTime);

        print "CURRENT ANGLE:$angle";
    }

= DESCRIPTION
    
    
The Kalman filter, also known as linear quadratic estimation (LQE), is an algorithm that uses a series 
of measurements observed over time, containing noise (random variations) and other inaccuracies, and 
produces estimates of unknown variables that tend to be more precise than those based on a single 
measurement alone.

Algorithm is recursive, which means it takes the output of its previous calculations as a factor in 
calculating the next step which improves its accuracy over time. The key to Kalman filters are two sensors
with different kind of accuracy issues in each. Sensor A or the state sensor might give in-accurate value
for a measurement on the whole but it doesn't drift. Sensor B or delta sensor gives gives much more accurate 
rate of change in value(or delta) but it drifts over time due to its small inaccuracies as it only measures
rate of change in value and not the actual value. Kalman filter uses this knowledge to fuse results from both
sensors to give a state value which is more accurate than state value received from any of these filters
alone.

An example of application for this is calculating orientation of objects using Gyroscopes and Accelerometers.

While Accelerometer is usually used to measure gravity it can be used to measure the inclination of a body 
with respect to the surface of earth along the x and y axis(not z axis as Z axis is usually facing the 
opposite direction as the force of gravity) by measuring the direction in which the force of gravity is 
applied.

Gyroscope measures the rate of rotation about one or all the axis of a body. while it gives fairly accurate 
estimation of the angular velocity, if we use it to calculate the current inclination based on the starting 
inclination and the angular velocity, there is a lot of drift, which means the gyroscope error will accumulate 
over time as we calculate newer angles based on previous angle and angular velocity and the error in angular 
velocity piles on.

A real life example of how Kalman filter works is while driving on a highway in a car. If you take the time 
passed since when your started driving and your estimated average speed every hour and use it to calculate 
the distance you have traveled your calculation will become more inaccurate as you drive on.

This is drift in value. However if you watch each milestone and calculate your current position using milestone
data and your speed since the last milestone your result will be much more accurate. That is approximately close
 to how Kalman filter works.

=end wikidoc

=cut
