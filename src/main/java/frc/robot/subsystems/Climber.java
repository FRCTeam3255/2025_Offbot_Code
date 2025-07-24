// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constClimber;
import frc.robot.RobotMap.mapClimber;

public class Climber extends SubsystemBase {
  TalonFX climberMotor;

  /** Creates a new Climber. */
  public Climber() {
    climberMotor = new TalonFX(mapClimber.CLIMBER_MOTOR_CAN);
    // Set default motor configurations if needed
    // e.g., climberLeftMotor.configFactoryDefault();
    climberMotor.getConfigurator().apply(constClimber.INTAKE_PIVOT_CONFIG);
  }

  public void setClimberMotorSpeed(double speed) {
    climberMotor.set(speed);
  }

  public void stopClimberMotor() {
    climberMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
