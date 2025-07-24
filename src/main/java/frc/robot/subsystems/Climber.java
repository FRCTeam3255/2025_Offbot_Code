// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constClimber;
import frc.robot.RobotMap.mapClimber;

public class Climber extends SubsystemBase {
  TalonFX cageCollectorMotor;

  /** Creates a new Climber. */
  public Climber() {
    cageCollectorMotor = new TalonFX(mapClimber.CLIMBER_MOTOR_CAN);
    // Set default motor configurations if needed
    // e.g., climberLeftMotor.configFactoryDefault();
    cageCollectorMotor.getConfigurator().apply(constClimber.CAGE_COLLECTOR_CONFIG);
  }

  public void setClimberMotorSpeed(double speed) {
    cageCollectorMotor.set(speed);
  }

  public void stopClimberMotor() {
    cageCollectorMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
