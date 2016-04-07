# NAME

Math::KalmanFilter - Kalman Filter(also known as Linear Quadratic Estimation) implementation for sensor fusion and such

<div>
    <p>
    <img src="https://img.shields.io/badge/perl-5.10+-brightgreen.svg" alt="Requires Perl 5.10+" />
    <a href="https://travis-ci.org/shantanubhadoria/perl-Math-KalmanFilter"><img src="https://api.travis-ci.org/shantanubhadoria/perl-Math-KalmanFilter.svg?branch=build/master" alt="Travis status" /></a>
    <a href="http://matrix.cpantesters.org/?dist=Math-KalmanFilter%200.07"><img src="https://badgedepot.code301.com/badge/cpantesters/Math-KalmanFilter/0.07" alt="CPAN Testers result" /></a>
    <a href="http://cpants.cpanauthors.org/dist/Math-KalmanFilter-0.07"><img src="https://badgedepot.code301.com/badge/kwalitee/Math-KalmanFilter/0.07" alt="Distribution kwalitee" /></a>
    <a href="https://gratipay.com/shantanubhadoria"><img src="https://img.shields.io/gratipay/shantanubhadoria.svg" alt="Gratipay" /></a>
    </p>
</div>

# VERSION

version 0.07

# SYNOPSIS

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

# DESCRIPTION

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

# ATTRIBUTES

## qAngle

    * default: 0.001 

## qBias

    * default: 0.003

## rMeasure 

    * default: 0.03

## bias

    * starting value(default): 0
    * recalculated(optimised) at each new sensor reading.

## covariance

This is the covariance matrix, it is stored as a 2d array ref
 \* starting value(default): \[\[0,0\],\[0,0\]\]
 \* recalculated(optimised) at each new sensor reading.

## angle 

Calculated angle

# METHODS

## getAngle

Calculate new state based on observed reading from state sensor, delta sensor and time elapsed since last reading.

# SUPPORT

## Bugs / Feature Requests

Please report any bugs or feature requests through github at 
[https://github.com/shantanubhadoria/perl-math-kalmanfilter/issues](https://github.com/shantanubhadoria/perl-math-kalmanfilter/issues).
You will be notified automatically of any progress on your issue.

## Source Code

This is open source software.  The code repository is available for
public review and contribution under the terms of the license.

[https://github.com/shantanubhadoria/perl-math-kalmanfilter](https://github.com/shantanubhadoria/perl-math-kalmanfilter)

    git clone git://github.com/shantanubhadoria/perl-math-kalmanfilter.git

# AUTHOR

Shantanu Bhadoria &lt;shantanu at cpan dott org>

# CONTRIBUTORS

- Shantanu Bhadoria <shantanu.bhadoria@gmail.com>
- Shantanu Bhadoria <shantanu@cpan.org>

# COPYRIGHT AND LICENSE

This software is copyright (c) 2016 by Shantanu Bhadoria.

This is free software; you can redistribute it and/or modify it under
the same terms as the Perl 5 programming language system itself.
