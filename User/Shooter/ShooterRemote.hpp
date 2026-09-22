#ifndef SHOOTER_REMOTE_HPP
#define SHOOTER_REMOTE_HPP

#include "ShooterCommand.hpp"

struct ShooterRemoteInput
{
    bool link_online = false;
    bool arm_switch_up = false;
    bool fire_hold = false;
    bool fire_once_button = false;
};

ShooterCommand BuildShooterCommand(const ShooterRemoteInput &input,
                                   bool &last_fire_once_button);

#endif
