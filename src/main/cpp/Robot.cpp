// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;

void Robot::RobotInit()
{
    motor_map[MotorID::FrontIntakeMotor] = new TalonFX((int)MotorID::FrontIntakeMotor, "canivore1");
    motor_map[MotorID::TunnelBeltMotor] = new TalonFX((int)MotorID::TunnelBeltMotor, "canivore1");
    motor_map[MotorID::UptakeMotor] = new TalonFX((int)MotorID::UptakeMotor, "canivore1");
    motor_map[MotorID::ShooterMotor] = new TalonFX((int)MotorID::ShooterMotor, "canivore1");
    motor_map[MotorID::ShooterMotorFollower] = new TalonFX((int)MotorID::ShooterMotorFollower, "canivore1");

    motor_map[MotorID::ShooterMotorFollower]->Follow(*motor_map[MotorID::ShooterMotor]);

    for (auto m : motor_map)
    {
        TalonFX* tfx = m.second;
        tfx->SetNeutralMode(NeutralMode::Coast);

        SupplyCurrentLimitConfiguration low_limit = SupplyCurrentLimitConfiguration(true, 10, 0, 0);
        SupplyCurrentLimitConfiguration high_limit = SupplyCurrentLimitConfiguration(true, 15, 0, 0);
        if (m.first == MotorID::ShooterMotor || m.first == MotorID::ShooterMotorFollower)
        {
            tfx->ConfigSupplyCurrentLimit(high_limit);
        }
        else
        {
            tfx->ConfigSupplyCurrentLimit(low_limit);
        }
    }
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
