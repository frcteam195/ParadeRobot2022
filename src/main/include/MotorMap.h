#pragma once

#include <ctre/Phoenix.h>
#include <map>

using namespace ctre::phoenix::motorcontrol::can;

enum class MotorID : int
{
    FrontIntakeMotor = 7,
    TunnelBeltMotor = 9,
    UptakeMotor = 11,
    ShooterMotor = 16,
    ShooterMotorFollower = 17
};

extern std::map<MotorID,TalonFX*> motor_map;