// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/Joystick.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;

frc::Joystick controller(0);
frc::Compressor compressor(frc::PneumaticsModuleType::CTREPCM);
frc::Solenoid intake_solenoid(frc::PneumaticsModuleType::CTREPCM, 3);
frc::Solenoid lower_hardstop_solenoid(frc::PneumaticsModuleType::CTREPCM, 0);
frc::Solenoid upper_hardstop_solenoid(frc::PneumaticsModuleType::CTREPCM, 5);

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

    motor_map[MotorID::FrontIntakeMotor]->SetInverted(true);
    motor_map[MotorID::TunnelBeltMotor]->SetInverted(true);
    motor_map[MotorID::UptakeMotor]->SetInverted(true);
    motor_map[MotorID::ShooterMotorFollower]->SetInverted(InvertType::OpposeMaster);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
    compressor.EnableDigital();
}

void Robot::TeleopPeriodic()
{
    intake_solenoid.Set(true);
    lower_hardstop_solenoid.Set(false);
    upper_hardstop_solenoid.Set(true);

    if (controller.GetRawButton(2))
    {
        motor_map[MotorID::FrontIntakeMotor]->Set(ControlMode::PercentOutput, 1);
        motor_map[MotorID::TunnelBeltMotor]->Set(ControlMode::PercentOutput, 1);
        motor_map[MotorID::UptakeMotor]->Set(ControlMode::PercentOutput, 1);
        motor_map[MotorID::ShooterMotor]->Set(ControlMode::PercentOutput, 0.55);
    }
    else if (controller.GetRawButton(3))
    {
        motor_map[MotorID::FrontIntakeMotor]->Set(ControlMode::PercentOutput, -1);
        motor_map[MotorID::TunnelBeltMotor]->Set(ControlMode::PercentOutput, -1);
        motor_map[MotorID::UptakeMotor]->Set(ControlMode::PercentOutput, -1);
        motor_map[MotorID::ShooterMotor]->Set(ControlMode::PercentOutput, 0);
    }
    else
    {
        for (auto m : motor_map)
        {
            m.second->Set(ControlMode::PercentOutput, 0);
        }
    }
}

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
