// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.RobotMap.mapRotors;

public class Rotors extends SubsystemBase {
  /** Creates a new Rotors. */
  @Logged

  TalonFX coralIntakeMotor;
  TalonFX algaeIntakeMotor;
  TalonFX cageCollectMotor;
  CANrange coralSensor;
  MotionMagicExpoVoltage positionRequest = new MotionMagicExpoVoltage(0);

  public Rotors() {
    coralIntakeMotor = new TalonFX(mapRotors.CORAL_INTAKE_CAN);
    algaeIntakeMotor = new TalonFX(mapRotors.INTAKE_ALGAE_CAN);
    coralSensor = new CANrange(mapRotors.CORAL_INTAKE_SENSOR);
    cageCollectMotor = new TalonFX(mapRotors.CAGE_COLLECTER_CAN);

    coralIntakeMotor.getConfigurator().apply(constRotors.CORAL_INTAKE_CONFIG);
    algaeIntakeMotor.getConfigurator().apply(constRotors.ALGAE_INTAKE_CONFIG);
    coralSensor.getConfigurator().apply(constRotors.CORAL_INTAKE_SENSOR_CONFIG);
    cageCollectMotor.getConfigurator().apply(constRotors.CLIMBER_CONFIG);
  }

  public boolean hasCoral() {
    return coralSensor.getIsDetected().getValue();
  }

  public boolean hasAlgae() {
    Current intakeCurrent = algaeIntakeMotor.getStatorCurrent().getValue();

    AngularVelocity intakeVelocity = algaeIntakeMotor.getVelocity().getValue();
    double intakeAcceleration = algaeIntakeMotor.getAcceleration().getValueAsDouble();

    Current intakeHasGamePieceCurrent = constRotors.ALGAE_INTAKE_HAS_GP_CURRENT;
    AngularVelocity intakeHasGamePieceVelocity = constRotors.ALGAE_INTAKE_HAS_GP_VELOCITY;

    if ((intakeCurrent.gte(intakeHasGamePieceCurrent))
        && (intakeVelocity.lte(intakeHasGamePieceVelocity))
        && (intakeAcceleration < 0)) {
      return true;
    } else {
      return false;
    }
  }

  public void setCoralIntakeMotorSpeed(double speed) {
    coralIntakeMotor.set(speed);
  }

  public void setAlgaeIntakeMotorSpeed(double speed) {
    algaeIntakeMotor.set(speed);
  }

  public void setAllIntake(double speed) {
    coralIntakeMotor.set(speed);
    algaeIntakeMotor.set(speed);
  }

  public void setClimberMotorPercentOutput(double speed) {
    cageCollectMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
