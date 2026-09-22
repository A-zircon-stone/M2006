#include "ShooterRemote.hpp"

#include <cassert>

int main()
{
    bool last_single_button = false;
    ShooterRemoteInput input{};

    ShooterCommand command = BuildShooterCommand(input, last_single_button);
    assert(!command.armed);
    assert(!command.friction_on);
    assert(!command.fire_once);
    assert(!command.fire_hold);
    assert(command.emergency_stop);

    input.link_online = true;
    input.arm_switch_up = true;
    input.fire_hold = true;
    command = BuildShooterCommand(input, last_single_button);
    assert(command.armed);
    assert(command.friction_on);
    assert(command.fire_hold);
    assert(!command.fire_once);
    assert(!command.emergency_stop);

    input.fire_hold = false;
    input.fire_once_button = true;
    command = BuildShooterCommand(input, last_single_button);
    assert(command.fire_once);

    command = BuildShooterCommand(input, last_single_button);
    assert(!command.fire_once);

    input.link_online = false;
    command = BuildShooterCommand(input, last_single_button);
    assert(command.emergency_stop);
    assert(!command.armed);
    assert(!command.fire_hold);
    return 0;
}
