#include "ShooterController.hpp"

#include <cassert>

static ShooterFeedback feedback(float left, float right, float feeder)
{
    return ShooterFeedback{left, right, feeder};
}

int main()
{
    ShooterController controller;
    ShooterCommand idle{};

    controller.reset(feedback(0.0f, 0.0f, 100.0f));
    controller.update(idle, feedback(0.0f, 0.0f, 100.0f));
    assert(controller.state() == ShooterState::Safe);
    assert(controller.output().left_speed_target == 0.0f);
    assert(controller.output().right_speed_target == 0.0f);
    assert(controller.output().safe_output);

    ShooterCommand spinup{true, true, false, false, false};
    controller.update(spinup, feedback(0.0f, 0.0f, 100.0f));
    assert(controller.state() == ShooterState::Spinup);
    assert(controller.output().left_speed_target == -6200.0f);
    assert(controller.output().right_speed_target == 6200.0f);
    assert(!controller.output().safe_output);

    for (int i = 0; i < 19; ++i)
    {
        controller.update(spinup, feedback(-6200.0f, 6200.0f, 100.0f));
    }
    assert(controller.state() == ShooterState::Spinup);

    controller.update(spinup, feedback(-6200.0f, 6200.0f, 100.0f));
    assert(controller.state() == ShooterState::Ready);

    ShooterCommand fire_once{true, true, true, false, false};
    controller.update(fire_once, feedback(-6200.0f, 6200.0f, 100.0f));
    assert(controller.state() == ShooterState::Firing);
    assert(controller.output().feeder_target == 145.0f);

    controller.update(ShooterCommand{true, true, false, false, false},
                      feedback(-6200.0f, 6200.0f, 100.0f));
    assert(controller.state() == ShooterState::Firing);
    controller.update(ShooterCommand{true, true, false, false, false},
                      feedback(-6200.0f, 6200.0f, 145.0f));
    controller.update(ShooterCommand{true, true, false, false, false},
                      feedback(-6200.0f, 6200.0f, 145.0f));
    assert(controller.state() == ShooterState::Cooldown);

    for (int i = 0; i < 10; ++i)
    {
        controller.update(ShooterCommand{true, true, false, true, false},
                          feedback(-6200.0f, 6200.0f, 145.0f));
    }
    assert(controller.state() == ShooterState::Ready);

    controller.update(ShooterCommand{true, true, false, true, false},
                      feedback(-6200.0f, 6200.0f, 145.0f));
    assert(controller.state() == ShooterState::Firing);
    assert(controller.output().feeder_target == 190.0f);

    controller.update(ShooterCommand{false, false, false, false, false},
                      feedback(-6200.0f, 6200.0f, 145.0f));
    assert(controller.state() == ShooterState::Safe);
    assert(controller.output().left_speed_target == 0.0f);
    assert(controller.output().right_speed_target == 0.0f);

    controller.update(ShooterCommand{true, true, false, false, false},
                      feedback(0.0f, 0.0f, 145.0f));
    for (int i = 0; i < 20; ++i)
    {
        controller.update(ShooterCommand{true, true, false, false, false},
                          feedback(-6200.0f, 6200.0f, 145.0f));
    }
    controller.update(ShooterCommand{true, true, true, false, false},
                      feedback(-6200.0f, 6200.0f, 145.0f));
    assert(controller.state() == ShooterState::Firing);
    for (int i = 0; i < 1000; ++i)
    {
        controller.update(ShooterCommand{true, true, false, false, false},
                          feedback(-6200.0f, 6200.0f, 145.0f));
    }
    assert(controller.state() == ShooterState::Safe);

    return 0;
}
