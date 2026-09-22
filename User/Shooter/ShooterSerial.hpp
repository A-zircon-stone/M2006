#ifndef SHOOTER_SERIAL_HPP
#define SHOOTER_SERIAL_HPP

#include "ShooterCommand.hpp"

void ShooterSerial_Init();
ShooterCommand ShooterSerial_GetCommand();

#endif
