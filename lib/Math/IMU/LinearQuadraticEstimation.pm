package Math::IMU::LinearQuadraticEstimation;

# PODNAME: Math::IMU::LinearQuadraticEstimation
# ABSTRACT: Linear Quadratic Estimation or Kalman Filter for Inertial Measurement Units
# COPYRIGHT
# VERSION

# Dependencies
use 5.010;
use Moose;

has qAngle => (
    is      => 'rw',
    default => 0.001,
);

has qBias => (
    is      => 'rw',
    default => 0.003,
);

has rMeasure => (
    is      => 'rw',
    default => 0.03,
);

has bias => (
    is      => 'rw',
    default => 0,
);

has covariance => (
    is      =>'rw',
    default => sub {
        return [
            [0,0],
            [0,0]
        ];
    }
);

has angle => (
    is      =>'rw',
);

sub getAngle {
    my ($self, $newAngle, $newRate, $deltaTime) = @_;
    
    my $covariance = $self->covariance;
    my $angle      = $self->angle;

    my $rate = $newAngle - $self->bias;
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
