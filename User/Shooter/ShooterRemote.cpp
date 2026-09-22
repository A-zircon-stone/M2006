#include "ShooterRemote.hpp"

ShooterCommand BuildShooterCommand(const ShooterRemoteInput &input,
                                   bool &last_fire_once_button)
{
    const bool fire_once_rising = input.fire_once_button && !last_fire_once_button;
    last_fire_once_button = input.fire_once_button;

    ShooterCommand command;
    command.armed = input.link_online && input.arm_switch_up;
    command.friction_on = command.armed;
    command.fire_once = input.link_online && fire_once_rising;
    command.fire_hold = input.link_online && input.fire_hold;
    command.emergency_stop = !input.link_online;
    return command;
}
