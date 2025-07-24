// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constIntake;
import frc.robot.RobotMap.mapIntake;

@Logged
public class Intake extends SubsystemBase {
  TalonFX intakePivotMotor;
  TalonFX algaeIntakeMotor;
  TalonFX coralIntakeMotor;

  CANrange coralSensor;

  private Angle lastDesiredAngle = Degrees.zero();

  PositionVoltage positionRequest = new PositionVoltage(0);
  VoltageOut voltageRequest = new VoltageOut(0);
  MotionMagicVoltage motionRequest = new MotionMagicVoltage(0);

  public boolean attemptingZeroing = false;
  public boolean hasZeroed = false;
  public boolean hasAlgaeOverride = false;
  public boolean hasCoralOverride = false;

  /** Creates a new Intake. */
  public Intake() {
    intakePivotMotor = new TalonFX(mapIntake.INTAKE_PIVOT_CAN); // Intake pivot motor
    algaeIntakeMotor = new TalonFX(mapIntake.INTAKE_ALGAE_CAN); // Algae intake motor
    coralIntakeMotor = new TalonFX(mapIntake.CORAL_LEFT_CAN); // Coral intake motor
    coralSensor = new CANrange(mapIntake.CORAL_SENSOR_CAN);

    // Set default motor configurations if needed
    // e.g., intakePivotMotor.configFactoryDefault();

    intakePivotMotor.getConfigurator().apply(constIntake.INTAKE_PIVOT_CONFIG);
    algaeIntakeMotor.getConfigurator().apply(constIntake.ALGAE_INTAKE_CONFIG);
    coralIntakeMotor.getConfigurator().apply(constIntake.CORAL_OUTTAKE_CONFIG);
    coralSensor.getConfigurator().apply(constIntake.CORAL_SENSOR_CONFIG);
  }

  public void setAlgaeIntakeMotorSpeed(double speed) {
    algaeIntakeMotor.set(speed);
  }

  public void setCoralIntakeMotorSpeed(double speed) {
    coralIntakeMotor.set(speed);
  }

  public void setPivotAngle(Angle setpoint) {
    if (hasAlgae()) {
      intakePivotMotor.setControl(motionRequest.withPosition(setpoint.in(Units.Rotation)).withSlot(1));
    } else {
      intakePivotMotor.setControl(motionRequest.withPosition(setpoint.in(Units.Rotation)).withSlot(0));
    }
    lastDesiredAngle = setpoint;
  }

  public AngularVelocity getPivotRotorVelocity() {
    return intakePivotMotor.getRotorVelocity().getValue();
  }

  public boolean isPivotRotorVelocityZero() {
    return getPivotRotorVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
  }

  public void setPivotVoltage(Voltage voltage) {
    intakePivotMotor.setControl(voltageRequest.withOutput(voltage));
  }

  public void setPivotSoftwareLimits(boolean reverseLimitEnable, boolean forwardLimitEnable) {
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

  public void resetPivotSensorPosition(Angle zeroedPos) {
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

  public boolean hasCoral() {
    if (hasCoralOverride) {
      return hasCoralOverride;
    }

    if (sensorSeesCoral()) {
      return true;
    } else {
      return false;
    }
  }

  public boolean sensorSeesCoral() {
    return coralSensor.getIsDetected().getValue();
  }

  public BooleanSupplier sensorSeesCoralSupplier() {
    return () -> coralSensor.getIsDetected().getValue();
  }

  public void setHasAlgaeOverride(boolean passedHasGamePiece) {
    hasAlgaeOverride = passedHasGamePiece;
  }

  public void algaeToggle() {
    this.hasAlgaeOverride = !hasAlgaeOverride;
  }

  public void setHasCoralOverride(boolean passedHasGamePiece) {
    hasCoralOverride = passedHasGamePiece;
  }

  public void coralToggle() {
    this.hasCoralOverride = !hasCoralOverride;
  }

  public double getAlgaeIntakeVoltage() {
    return algaeIntakeMotor.getMotorVoltage().getValueAsDouble();
  }

  public double getCoralIntakeVoltage() {
    return coralIntakeMotor.getMotorVoltage().getValueAsDouble();
  }

  public void setAlgaeIntakeVoltage(double voltage) {
    algaeIntakeMotor.setVoltage(voltage);
  }

  public void setCoralIntakeVoltage(double voltage) {
    coralIntakeMotor.setVoltage(voltage);
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
