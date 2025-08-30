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

@Logged
public class Rotors extends SubsystemBase {
  /** Creates a new Rotors. */

  TalonFX coralIntakeLeftMotor;
  TalonFX coralIntakeRightMotor;
  TalonFX algaeIntakeMotor;
  TalonFX cageCollectMotor;
  CANrange coralMidSensor;
  CANrange coralLeftSensor;
  CANrange coralRightSensor;

  MotionMagicExpoVoltage positionRequest = new MotionMagicExpoVoltage(0);

  public Rotors() {
    coralIntakeLeftMotor = new TalonFX(mapRotors.CORAL_INTAKE_LEFT_CAN);
    coralIntakeRightMotor = new TalonFX(mapRotors.CORAL_INTAKE_RIGHT_CAN);
    algaeIntakeMotor = new TalonFX(mapRotors.INTAKE_ALGAE_CAN);
    coralMidSensor = new CANrange(mapRotors.CORAL_MID_SENSOR);
    cageCollectMotor = new TalonFX(mapRotors.CAGE_COLLECTER_CAN);
    coralLeftSensor = new CANrange(mapRotors.CORAL_LEFT_SENSOR);
    coralRightSensor = new CANrange(mapRotors.CORAL_RIGHT_SENSOR);

    coralIntakeLeftMotor.getConfigurator().apply(constRotors.CORAL_INTAKE_CONFIG);
    coralIntakeRightMotor.getConfigurator().apply(constRotors.CORAL_INTAKE_CONFIG);
    algaeIntakeMotor.getConfigurator().apply(constRotors.ALGAE_INTAKE_CONFIG);
    coralMidSensor.getConfigurator().apply(constRotors.CORAL_INTAKE_SENSOR_CONFIG);
    coralLeftSensor.getConfigurator().apply(constRotors.CORAL_INTAKE_SENSOR_CONFIG);
    coralRightSensor.getConfigurator().apply(constRotors.CORAL_INTAKE_SENSOR_CONFIG);
    cageCollectMotor.getConfigurator().apply(constRotors.CLIMBER_CONFIG);
  }

  public boolean hasCoral() {
    return coralMidSensor.getIsDetected().getValue() &&
        !coralLeftSensor.getIsDetected().getValue() &&
        !coralRightSensor.getIsDetected().getValue();
  }

  public boolean hasL1Coral() {
    return (coralMidSensor.getIsDetected().getValue() &&
        coralLeftSensor.getIsDetected().getValue()) ||
        (coralMidSensor.getIsDetected().getValue() &&
            coralRightSensor.getIsDetected().getValue());
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
    coralIntakeLeftMotor.set(speed);
    coralIntakeRightMotor.set(-speed);
  }

  public void setCoralIntakeL1Speed(double speed) {
    coralIntakeLeftMotor.set(speed);
    coralIntakeRightMotor.set(speed);
  }

  public void setAlgaeIntakeMotorSpeed(double speed) {
    algaeIntakeMotor.set(speed);
  }

  public void setAllIntake(double speed) {
    coralIntakeLeftMotor.set(speed);
    coralIntakeRightMotor.set(speed);
    algaeIntakeMotor.set(speed);
  }

  public void setClimberMotorPercentOutput(double speed) {
    cageCollectMotor.set(speed);
  }

  public boolean isCageLatched() {
    Current collectorCurrent = cageCollectMotor.getStatorCurrent().getValue();
    if (collectorCurrent.gt(constRotors.COLLECTOR_HAS_CAGE_CURRENT)) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
