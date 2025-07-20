// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapClimber;

public class Climber extends SubsystemBase {
  TalonFX climberLeftMotor;

  /** Creates a new Climber. */
  public Climber() {
    climberLeftMotor = new TalonFX(mapClimber.CLIMBER_LEFT_CAN); // Climber left motor, using CAN ID 31 as per RobotMap
    // Set default motor configurations if needed
    // e.g., climberLeftMotor.configFactoryDefault();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
