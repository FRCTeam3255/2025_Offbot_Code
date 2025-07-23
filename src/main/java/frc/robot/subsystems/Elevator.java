// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.constElevator;
import frc.robot.RobotMap.mapElevator;

@Logged
public class Elevator extends SubsystemBase {
  private TalonFX leftElevatorMotorFollower;
  private TalonFX rightElevatorMotorLeader;
  private TalonFX leftElevatorPivotMotorFollower;
  private TalonFX rightElevatorPivotMotorLeader;

  private Distance lastDesiredElevatorPosition;
  private Angle lastDesiredElevatorPivotAngle;

  Distance currentLeftElevatorPosition = Units.Inches.of(0);
  Distance currentRightElevatorPosition = Units.Inches.of(0);

  PositionVoltage elevatorPositionRequest;
  VoltageOut elevatorVoltageRequest = new VoltageOut(0);
  PositionVoltage elevatorPivotPositionRequest;
  VoltageOut elevatorPivotVoltageRequest = new VoltageOut(0);


  public boolean attemptingElevatorZeroing = false;
  public boolean hasZeroedElevator = false;
  public boolean attemptingElevatorPivotZeroing = false;
  public boolean hasZeroedElevatorPivot = false;

  MotionMagicExpoVoltage elevatorMotionRequest;
  MotionMagicExpoVoltage elevatorPivotMotionRequest;

  /** Creates a new Elevator. */
  public Elevator() {
    leftElevatorMotorFollower = new TalonFX(mapElevator.ELEVATOR_LEFT_CAN);
    rightElevatorMotorLeader = new TalonFX(mapElevator.ELEVATOR_RIGHT_CAN);
    leftElevatorPivotMotorFollower = new TalonFX(mapElevator.ELEVATOR_LEFT_PIVOT_CAN);
    rightElevatorPivotMotorLeader = new TalonFX(mapElevator.ELEVATOR_RIGHT_PIVOT_CAN);

    lastDesiredElevatorPosition = Units.Inches.of(0);
    elevatorVoltageRequest = new VoltageOut(0);
    elevatorMotionRequest = new MotionMagicExpoVoltage(0);
    lastDesiredElevatorPivotAngle = Degrees.of(0);
    elevatorVoltageRequest = new VoltageOut(0);
    elevatorMotionRequest = new MotionMagicExpoVoltage(0);

    rightElevatorMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    leftElevatorMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    leftElevatorPivotMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_PIVOT_CONFIG);
    rightElevatorPivotMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_PIVOT_CONFIG);
  }

  public Distance getElevatorPosition() {
    if (Robot.isSimulation()) {
      return getLastDesiredElevatorPosition();
    }
    return Units.Inches.of(rightElevatorMotorLeader.getPosition().getValueAsDouble());
  }

  public Angle getElevatorPivotAngle() {
    if (Robot.isSimulation()) {
      return getLastDesiredElevatorPivotAngle();
    }
    return Units.Degrees.of(rightElevatorPivotMotorLeader.getPosition().getValueAsDouble());
  }

  public boolean elevatorAtDesiredPosition() {
    return isElevatorAtSetPointWithTolerance(getLastDesiredElevatorPosition(), Constants.constElevator.DEADZONE_DISTANCE);
  }

  public boolean elevatorPivotAtDesiredAngle() {
    return isElevatorPivotAtSetPointWithTolerance(getLastDesiredElevatorPivotAngle(), Constants.constElevator.DEADZONE_ANGLE);
  }

  public boolean elevatorIsAtSpecificSetpoint(Distance setpoint) {
    return isElevatorAtSetPointWithTolerance(setpoint, Constants.constElevator.DEADZONE_DISTANCE);
  }

  public boolean elevatorPivotIsAtSpecificSetpoint(Angle setpoint) {
    return isElevatorPivotAtSetPointWithTolerance(setpoint, Constants.constElevator.DEADZONE_ANGLE);
  }

  public boolean isElevatorAtSetPointWithTolerance(Distance position, Distance tolerance) {
    if (Robot.isSimulation()) {
      return true;
    }
    return (getElevatorPosition()
      .compareTo(position.minus(tolerance)) > 0) &&
      getElevatorPosition().compareTo(position.plus(tolerance)) < 0;
    }

    public boolean isElevatorPivotAtSetPointWithTolerance(Angle position, Angle tolerance) {
    if (Robot.isSimulation()) {
      return true;
    }
    return (getElevatorPivotAngle()
      .compareTo(position.minus(tolerance)) > 0) &&
      getElevatorPivotAngle().compareTo(position.plus(tolerance)) < 0;
    }

    public boolean isAtAnyCoralScoringPosition() {
    if (elevatorIsAtSpecificSetpoint(constElevator.CORAL_L1_HEIGHT) && elevatorPivotIsAtSpecificSetpoint(constElevator.CORAL_L1_ANGLE) ||
        elevatorIsAtSpecificSetpoint(constElevator.CORAL_L2_HEIGHT) && elevatorPivotIsAtSpecificSetpoint(constElevator.CORAL_L2_ANGLE) ||
        elevatorIsAtSpecificSetpoint(constElevator.CORAL_L3_HEIGHT) && elevatorPivotIsAtSpecificSetpoint(constElevator.CORAL_L3_ANGLE) ||
        elevatorIsAtSpecificSetpoint(constElevator.CORAL_L4_HEIGHT) && elevatorPivotIsAtSpecificSetpoint(constElevator.CORAL_L4_ANGLE)) {
      return true;
    }
    return false;
  }

  public boolean isAtAnyAlgaeScoringPosition() {
    if (elevatorIsAtSpecificSetpoint(constElevator.ALGAE_PREP_NET) && elevatorPivotIsAtSpecificSetpoint(constElevator.ALGAE_PREP_NET_ANGLE)) {
      return true;
    }
    return false;
  }

  public AngularVelocity getElevatorRotorVelocity() {
    return rightElevatorMotorLeader.getRotorVelocity().getValue();
  }

  public AngularVelocity getElevatorPivotRotorVelocity() {
    return rightElevatorPivotMotorLeader.getRotorVelocity().getValue();
  }

  public Distance getLastDesiredElevatorPosition() {
    return lastDesiredElevatorPosition;
  }

  public Angle getLastDesiredElevatorPivotAngle() {
    return lastDesiredElevatorPivotAngle;
  }

  public void setElevatorCoastMode(Boolean coastMode) {
    if (coastMode) {
      rightElevatorMotorLeader.getConfigurator().apply(constElevator.COAST_MODE_CONFIGURATION);
      leftElevatorMotorFollower.getConfigurator().apply(constElevator.COAST_MODE_CONFIGURATION);
    } else {
      rightElevatorMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
      leftElevatorMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    }
  }

  public void setElevatorPivotCoastMode(Boolean coastMode) {
    if (coastMode) {
      rightElevatorPivotMotorLeader.getConfigurator().apply(constElevator.COAST_MODE_CONFIGURATION);
      leftElevatorPivotMotorFollower.getConfigurator().apply(constElevator.COAST_MODE_CONFIGURATION);
    } else {
      rightElevatorPivotMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_PIVOT_CONFIG);
      leftElevatorPivotMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_PIVOT_CONFIG);
    }
  }

  public boolean isElevatorRotorVelocityZero() {
    return getElevatorRotorVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
  }
  
  public boolean isElevatorPivotRotorVelocityZero() {
    return getElevatorPivotRotorVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
  }

  public void setElevatorPosition(Distance height) {
    rightElevatorMotorLeader.setControl(elevatorMotionRequest.withPosition(height.in(Units.Inches)));
    leftElevatorMotorFollower.setControl(new Follower(rightElevatorMotorLeader.getDeviceID(), true));
    lastDesiredElevatorPosition = height;
  }

  public void setElevatorPivotAngle(Angle angle) {
    rightElevatorPivotMotorLeader.setControl(elevatorPivotMotionRequest.withPosition(angle.in(Units.Degrees)));
    leftElevatorPivotMotorFollower.setControl(new Follower(rightElevatorPivotMotorLeader.getDeviceID(), true));
    lastDesiredElevatorPivotAngle = angle;
  }

  public void setElevatorNeutral() {
    rightElevatorMotorLeader.setControl(new NeutralOut());
    leftElevatorMotorFollower.setControl(new NeutralOut());
  }

  public void setElevatorPivotNeutral() {
    rightElevatorPivotMotorLeader.setControl(new NeutralOut());
    leftElevatorPivotMotorFollower.setControl(new NeutralOut());
  }

  public void setElevatorVoltage(Voltage voltage) {
    rightElevatorMotorLeader.setControl(elevatorVoltageRequest.withOutput(voltage));
    leftElevatorMotorFollower.setControl(new Follower(rightElevatorMotorLeader.getDeviceID(), true));
  }

  public void setElevatorPivotVoltage(Voltage voltage) {
    rightElevatorPivotMotorLeader.setControl(elevatorPivotVoltageRequest.withOutput(voltage));
    leftElevatorPivotMotorFollower.setControl(new Follower(rightElevatorPivotMotorLeader.getDeviceID(), true));
  }

  public void setElevatorSoftwareLimitsEnable(boolean reverseLimitEnable, boolean forwardLimitEnable) {
    constElevator.ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverseLimitEnable;
    constElevator.ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = forwardLimitEnable;

    rightElevatorMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    leftElevatorMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
  }

  public void setElevatorPivotSoftwareLimitsEnable(boolean reverseLimitEnable, boolean forwardLimitEnable) {
    constElevator.ELEVATOR_PIVOT_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverseLimitEnable;
    constElevator.ELEVATOR_PIVOT_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = forwardLimitEnable;

    rightElevatorMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_PIVOT_CONFIG);
    leftElevatorMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_PIVOT_CONFIG);
  }

  public void setElevatorSoftwareLimits(double reverseLimit, double forwardLimit) {
    constElevator.ELEVATOR_PIVOT_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit;
    constElevator.ELEVATOR_PIVOT_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;

    rightElevatorPivotMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    leftElevatorPivotMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
  }

  public void setElevatorPivotSoftwareLimits(double reverseLimit, double forwardLimit) {
    constElevator.ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit;
    constElevator.ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;

    rightElevatorPivotMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    leftElevatorPivotMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
  }

  public void resetElevatorSensorPosition(Distance setpoint) {
    rightElevatorMotorLeader.setPosition(setpoint.in(Inches));
    leftElevatorMotorFollower.setPosition(setpoint.in(Inches));
  }

  public void resetElevatorPivotSensorPosition(Angle setpoint) {
    rightElevatorPivotMotorLeader.setPosition(setpoint.in(Degrees));
    leftElevatorPivotMotorFollower.setPosition(setpoint.in(Degrees));
  }

  @Override
  public void periodic() {
  }
}