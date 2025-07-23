// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constIntake;
import frc.robot.RobotMap.mapIntake;

public class Intake extends SubsystemBase {
  TalonFX intakePivotMotor;
  TalonFX coralLeftMotor;
  TalonFX coralRightMotor;
  TalonFX algaeIntakeMotor;

  private Angle lastDesiredAngle = Degrees.zero();

  PositionVoltage positionRequest = new PositionVoltage(0);
  VoltageOut voltageRequest = new VoltageOut(0);
  MotionMagicVoltage motionRequest = new MotionMagicVoltage(0);

  public boolean attemptingZeroing = false;
  public boolean hasZeroed = false;
  public boolean hasAlgaeOverride = false;
  public boolean YEET = false;

  /** Creates a new Intake. */
  public Intake() {
    intakePivotMotor = new TalonFX(mapIntake.INTAKE_PIVOT_CAN); // Intake pivot motor
    coralLeftMotor = new TalonFX(mapIntake.CORAL_LEFT_CAN); // Coral left intake motor
    coralRightMotor = new TalonFX(mapIntake.CORAL_RIGHT_CAN); // Coral right intake motor
    algaeIntakeMotor = new TalonFX(mapIntake.INTAKE_ALGAE_CAN); // Algae intake motor

    // Set default motor configurations if needed
    // e.g., intakePivotMotor.configFactoryDefault();

  }

  public void setAlgaeIntakeMotor(double speed) {
    algaeIntakeMotor.set(speed);
  }

  public void setAlgaePivotAngle(Angle setpoint) {
    if (hasAlgae()) {
      intakePivotMotor.setControl(motionRequest.withPosition(setpoint.in(Units.Rotation)).withSlot(1));
    } else {
      intakePivotMotor.setControl(motionRequest.withPosition(setpoint.in(Units.Rotation)).withSlot(0));
    }
    lastDesiredAngle = setpoint;
  }

  public AngularVelocity getRotorVelocity() {
    return intakePivotMotor.getRotorVelocity().getValue();
  }

  public boolean isRotorVelocityZero() {
    return getRotorVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
  }

  public void setVoltage(Voltage voltage) {
    intakePivotMotor.setControl(voltageRequest.withOutput(voltage));
  }

  public void setSoftwareLimits(boolean reverseLimitEnable, boolean forwardLimitEnable) {
    constIntake.INTAKE_PIVOT_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverseLimitEnable;
    constIntake.INTAKE_PIVOT_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = forwardLimitEnable;

    intakePivotMotor.getConfigurator().apply(constIntake.INTAKE_PIVOT_CONFIG);
  }

  public Angle getPivotAngle() {
    return intakePivotMotor.getPosition().getValue();
  }

  public Angle getLastDesiredPivotAngle() {
    return lastDesiredAngle;
  }

  public void resetSensorPosition(Angle zeroedPos) {
    intakePivotMotor.setPosition(zeroedPos);
  }

  public boolean hasAlgae() {
    Current intakeCurrent = algaeIntakeMotor.getStatorCurrent().getValue();

    AngularVelocity intakeVelocity = algaeIntakeMotor.getVelocity().getValue();
    double intakeAcceleration = algaeIntakeMotor.getAcceleration().getValueAsDouble();

    Current intakeHasGamePieceCurrent = constIntake.ALGAE_INTAKE_HAS_GP_CURRENT;
    AngularVelocity intakeHasGamePieceVelocity = constIntake.ALGAE_INTAKE_HAS_GP_VELOCITY;

    if (hasAlgaeOverride) {
      return hasAlgaeOverride;
    }

    if ((intakeCurrent.gte(intakeHasGamePieceCurrent))
        && (intakeVelocity.lte(intakeHasGamePieceVelocity))
        && (intakeAcceleration < 0)) {
      return true;
    } else {
      return false;
    }
  }

  public void setHasAlgaeOverride(boolean passedHasGamePiece) {
    hasAlgaeOverride = passedHasGamePiece;
  }

  public void algaeToggle() {
    this.hasAlgaeOverride = !hasAlgaeOverride;
  }

  public double getAlgaeIntakeVoltage() {
    return algaeIntakeMotor.getMotorVoltage().getValueAsDouble();
  }

  public void setAlgaeIntakeVoltage(double voltage) {
    algaeIntakeMotor.setVoltage(voltage);

  }

  public boolean isAtSetPoint() {
    return (getPivotAngle()
        .compareTo(getLastDesiredPivotAngle().minus(constIntake.DEADZONE_DISTANCE)) > 0) &&
        getPivotAngle().compareTo(getLastDesiredPivotAngle().plus(constIntake.DEADZONE_DISTANCE)) < 0;
  }

  public boolean isAtSpecificSetpoint(Angle setpoint) {
    return (getPivotAngle()
        .compareTo(setpoint.minus(constIntake.DEADZONE_DISTANCE)) > 0) &&
        getPivotAngle().compareTo(setpoint.plus(constIntake.DEADZONE_DISTANCE)) < 0;
  }

  public boolean isAtAnyAlgaeScoringPosition() {
    if (isAtSpecificSetpoint(constIntake.PREP_NET_PIVOT_POSITION) ||
        isAtSpecificSetpoint(constIntake.PREP_PROCESSOR_PIVOT_POSITION)) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {

  }
}
