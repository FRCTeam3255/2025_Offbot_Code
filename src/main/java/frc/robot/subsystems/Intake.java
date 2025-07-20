// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapIntake;

public class Intake extends SubsystemBase {
  TalonFX intakePivotMotor;
  TalonFX coralLeftMotor;
  TalonFX coralRightMotor;
  TalonFX algaeIntakeMotor;

  /** Creates a new Intake. */
  public Intake() {
    intakePivotMotor = new TalonFX(mapIntake.INTAKE_PIVOT_CAN); // Intake pivot motor
    coralLeftMotor = new TalonFX(mapIntake.CORAL_LEFT_CAN); // Coral left intake motor
    coralRightMotor = new TalonFX(mapIntake.CORAL_RIGHT_CAN); // Coral right intake motor
    algaeIntakeMotor = new TalonFX(mapIntake.INTAKE_ALGAE_CAN); // Algae intake motor

    // Set default motor configurations if needed
    // e.g., intakePivotMotor.configFactoryDefault();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
