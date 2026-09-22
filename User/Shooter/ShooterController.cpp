#include "ShooterController.hpp"

#include <cmath>

void ShooterController::reset(const ShooterFeedback &feedback)
{
    _state = ShooterState::Safe;
    _ready_samples = 0;
    _arrival_samples = 0;
    _cooldown_samples = 0;
    _firing_samples = 0;
    _last_fire_once = false;
    enterSafe(feedback.feeder_position);
}

void ShooterController::update(const ShooterCommand &command,
                               const ShooterFeedback &feedback)
{
    const bool fire_once_rising = command.fire_once && !_last_fire_once;
    _last_fire_once = command.fire_once;

    if (command.emergency_stop || !command.armed || !command.friction_on)
    {
        enterSafe(feedback.feeder_position);
        return;
    }

    switch (_state)
    {
        case ShooterState::Safe:
            beginSpinup();
            break;

        case ShooterState::Spinup:
            updateSpinup(feedback);
            break;

        case ShooterState::Ready:
            setFrictionTargets();
            if (fire_once_rising || command.fire_hold)
            {
                startShot();
            }
            break;

        case ShooterState::Firing:
            setFrictionTargets();
            updateFiring(feedback);
            break;

        case ShooterState::Cooldown:
            setFrictionTargets();
            updateCooldown();
            break;
    }
}

ShooterState ShooterController::state() const
{
    return _state;
}

const ShooterOutput &ShooterController::output() const
{
    return _output;
}

void ShooterController::enterSafe(float feeder_position)
{
    _state = ShooterState::Safe;
    _ready_samples = 0;
    _arrival_samples = 0;
    _cooldown_samples = 0;
    _firing_samples = 0;
    _output.left_speed_target = 0.0f;
    _output.right_speed_target = 0.0f;
    _output.feeder_target = feeder_position;
    _output.safe_output = true;
}

void ShooterController::beginSpinup()
{
    _state = ShooterState::Spinup;
    _ready_samples = 0;
    _arrival_samples = 0;
    _firing_samples = 0;
    setFrictionTargets();
}

void ShooterController::updateSpinup(const ShooterFeedback &feedback)
{
    setFrictionTargets();

    if (inReadyBand(feedback.left_speed, false) &&
        inReadyBand(feedback.right_speed, true))
    {
        ++_ready_samples;
        if (_ready_samples >= kReadySamples)
        {
            _state = ShooterState::Ready;
            _ready_samples = 0;
        }
    }
    else
    {
        _ready_samples = 0;
    }
}

void ShooterController::startShot()
{
    _state = ShooterState::Firing;
    _output.feeder_target += kFeederStep;
    _arrival_samples = 0;
    _firing_samples = 0;
}

void ShooterController::updateFiring(const ShooterFeedback &feedback)
{
    ++_firing_samples;

    if (std::fabs(feedback.feeder_position - _output.feeder_target) <=
        kFeederPositionTolerance)
    {
        ++_arrival_samples;
        if (_arrival_samples >= kArrivalSamples)
        {
            _state = ShooterState::Cooldown;
            _cooldown_samples = 0;
            _arrival_samples = 0;
        }
    }
    else
    {
        _arrival_samples = 0;
    }

    if (_firing_samples >= kFiringTimeoutSamples &&
        _state == ShooterState::Firing)
    {
        enterSafe(feedback.feeder_position);
    }
}

void ShooterController::updateCooldown()
{
    ++_cooldown_samples;
    if (_cooldown_samples >= kCooldownSamples)
    {
        _state = ShooterState::Ready;
        _cooldown_samples = 0;
    }
}

void ShooterController::setFrictionTargets()
{
    _output.left_speed_target = kLeftSpeedTarget;
    _output.right_speed_target = kRightSpeedTarget;
    _output.safe_output = false;
}

bool ShooterController::inReadyBand(float speed, bool positive)
{
    const float magnitude = positive ? speed : -speed;
    return magnitude >= kReadySpeedMin && magnitude <= kReadySpeedMax;
}
