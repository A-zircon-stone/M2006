#ifndef SHOOTER_COMMAND_HPP
#define SHOOTER_COMMAND_HPP

struct ShooterCommand
{
    bool armed = false;
    bool friction_on = false;
    bool fire_once = false;
    bool fire_hold = false;
    bool emergency_stop = false;
};

#endif
