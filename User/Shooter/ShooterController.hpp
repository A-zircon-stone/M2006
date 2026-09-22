#ifndef SHOOTER_CONTROLLER_HPP
#define SHOOTER_CONTROLLER_HPP

#include "ShooterCommand.hpp"

enum class ShooterState
{
    Safe,
    Spinup,
    Ready,
    Firing,
    Cooldown
};

struct ShooterFeedback
{
    float left_speed = 0.0f;
    float right_speed = 0.0f;
    float feeder_position = 0.0f;
};

struct ShooterOutput
{
    float left_speed_target = 0.0f;
    float right_speed_target = 0.0f;
    float feeder_target = 0.0f;
    bool safe_output = true;
};

class ShooterController
{
public:
    void reset(const ShooterFeedback &feedback);
    void update(const ShooterCommand &command, const ShooterFeedback &feedback);
    ShooterState state() const;
    const ShooterOutput &output() const;

private:
    static constexpr float kLeftSpeedTarget = -6200.0f;
    static constexpr float kRightSpeedTarget = 6200.0f;
    static constexpr float kReadySpeedMin = 5900.0f;
    static constexpr float kReadySpeedMax = 6300.0f;
    static constexpr float kFeederStep = 45.0f;
    static constexpr float kFeederPositionTolerance = 1.0f;
    static constexpr int kReadySamples = 20;
    static constexpr int kArrivalSamples = 2;
    static constexpr int kCooldownSamples = 10;
    static constexpr int kFiringTimeoutSamples = 1000;

    ShooterState _state = ShooterState::Safe;
    ShooterOutput _output{};
    int _ready_samples = 0;
    int _arrival_samples = 0;
    int _cooldown_samples = 0;
    int _firing_samples = 0;
    bool _last_fire_once = false;

    void enterSafe(float feeder_position);
    void beginSpinup();
    void updateSpinup(const ShooterFeedback &feedback);
    void startShot();
    void updateFiring(const ShooterFeedback &feedback);
    void updateCooldown();
    void setFrictionTargets();
    static bool inReadyBand(float speed, bool positive);
};

#endif
