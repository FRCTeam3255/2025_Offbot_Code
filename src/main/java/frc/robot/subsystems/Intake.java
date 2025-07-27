// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.RobotMap.mapIntake;

public class Intake extends SubsystemBase {
  TalonFX intakePivotMotor;
  TalonFX coralIntakeMotor;
  TalonFX algaeIntakeMotor;
  CANrange coralSensor;
  public boolean hasCoral = false;
  public boolean hasAlgae = false;

  /** Creates a new Intake. */
  public Intake() {
    intakePivotMotor = new TalonFX(mapIntake.INTAKE_PIVOT_CAN); // Intake pivot motor
    coralIntakeMotor = new TalonFX(mapIntake.CORAL_INTAKE_CAN); // Coral left intake motor
    algaeIntakeMotor = new TalonFX(mapIntake.INTAKE_ALGAE_CAN); // Algae intake motor

    // Set default motor configurations if needed
    // e.g., intakePivotMotor.configFactoryDefault();
  }

  public boolean hasCoral() {
    return hasCoral;
  }

  public boolean hasAlgae() {
    Current intakeCurrent = algaeIntakeMotor.getStatorCurrent().getValue();

    AngularVelocity intakeVelocity = algaeIntakeMotor.getVelocity().getValue();
    double intakeAcceleration = algaeIntakeMotor.getAcceleration().getValueAsDouble();

    Current intakeHasGamePieceCurrent = constIntake.ALGAE_INTAKE_HAS_GP_CURRENT;
    AngularVelocity intakeHasGamePieceVelocity = constIntake.ALGAE_INTAKE_HAS_GP_VELOCITY;

    if ((intakeCurrent.gte(intakeHasGamePieceCurrent))
        && (intakeVelocity.lte(intakeHasGamePieceVelocity))
        && (intakeAcceleration < 0)) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
