// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismPositionGroup;
import frc.robot.Constants.constMotion;
import frc.robot.Robot;
import frc.robot.RobotMap.*;

@Logged
public class Motion extends SubsystemBase {
  /** Creates a new Motion. */
  TalonFX leftLiftMotorFollower;
  TalonFX rightLiftMotorLeader;
  TalonFX leftPivotMotorFollower;
  TalonFX rightPivotMotorLeader;
  TalonFX wristPivotMotor;

  private Angle elevatorPivotLastDesiredAngle = Degrees.zero();
  private Angle wristLastDesiredAngle = Degrees.zero();
  private Distance elevatorLiftLastDesiredPosition = Units.Inches.zero();
  MotionMagicExpoVoltage positionRequest = new MotionMagicExpoVoltage(0);
  public boolean attemptingZeroing = false;
  public boolean hasLiftZeroed = false;
  public boolean hasPivotZeroed = false;
  public boolean hasWristZeroed = false;

  public Motion() {
    leftLiftMotorFollower = new TalonFX(mapMotion.LEFT_LIFT_CAN);
    rightLiftMotorLeader = new TalonFX(mapMotion.RIGHT_LIFT_CAN);
    leftPivotMotorFollower = new TalonFX(mapMotion.LEFT_PIVOT_CAN);
    rightPivotMotorLeader = new TalonFX(mapMotion.RIGHT_PIVOT_CAN);
    wristPivotMotor = new TalonFX(mapMotion.INTAKE_PIVOT_CAN);

    elevatorLiftLastDesiredPosition = Units.Inches.of(0);
    // Set default motor configurations if needed
    // e.g., elevatorLeftMotor.configFactoryDefault();
    leftLiftMotorFollower.getConfigurator().apply(constMotion.LIFT_CONFIG);
    rightLiftMotorLeader.getConfigurator().apply(constMotion.LIFT_CONFIG);
    leftPivotMotorFollower.getConfigurator().apply(constMotion.ELEVATOR_PIVOT_CONFIG);
    rightPivotMotorLeader.getConfigurator().apply(constMotion.ELEVATOR_PIVOT_CONFIG);
    wristPivotMotor.getConfigurator().apply(constMotion.WRIST_CONFIG);
  }

  public void setLiftPosition(Distance height) {
    rightLiftMotorLeader.setControl(positionRequest.withPosition(height.in(Units.Inches)));
    leftLiftMotorFollower.setControl(new Follower(rightLiftMotorLeader.getDeviceID(), true));
    elevatorLiftLastDesiredPosition = height;
  }

  private void setElevatorPivotAngle(Angle angle) {
    rightPivotMotorLeader.setControl(positionRequest.withPosition(angle.in(Degrees)));
    leftPivotMotorFollower.setControl(new Follower(rightPivotMotorLeader.getDeviceID(), true));
    elevatorPivotLastDesiredAngle = angle;
  }

  private void setWristPivotAngle(Angle angle) {
    wristPivotMotor.setControl(positionRequest.withPosition(angle.in(Degrees)));
    wristLastDesiredAngle = angle;
  }

  public void setAllPosition(MechanismPositionGroup positionGroup) {
    setLiftPosition(positionGroup.liftHeight);
    setElevatorPivotAngle(positionGroup.pivotAngle);
    setWristPivotAngle(positionGroup.wristAngle);
  }

  public void setLiftCoastMode(boolean coastMode) {
    if (coastMode) {
      rightLiftMotorLeader.setNeutralMode(NeutralModeValue.Coast);
      leftLiftMotorFollower.setNeutralMode(NeutralModeValue.Coast);
    } else {
      rightLiftMotorLeader.setNeutralMode(NeutralModeValue.Brake);
      leftLiftMotorFollower.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  public void setPivotCoastMode(boolean coastMode) {
    if (coastMode) {
      rightPivotMotorLeader.setNeutralMode(NeutralModeValue.Coast);
      leftPivotMotorFollower.setNeutralMode(NeutralModeValue.Coast);
    } else {
      rightPivotMotorLeader.setNeutralMode(NeutralModeValue.Brake);
      leftPivotMotorFollower.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  public void setWristCoastMode(boolean coastMode) {
    if (coastMode) {
      wristPivotMotor.setNeutralMode(NeutralModeValue.Coast);
    } else {
      wristPivotMotor.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  public Distance getLiftPosition() {
    if (Robot.isSimulation()) {
      return elevatorLiftLastDesiredPosition;
    }
    return Units.Inches.of(rightLiftMotorLeader.getPosition().getValueAsDouble());
  }

  public Angle getPivotAngle() {
    if (Robot.isSimulation()) {
      return elevatorPivotLastDesiredAngle;
    }
    return rightPivotMotorLeader.getPosition().getValue();
  }

  public Angle getWristAngle() {
    if (Robot.isSimulation()) {
      return wristLastDesiredAngle;
    }
    return Degrees.of(wristPivotMotor.getPosition().getValueAsDouble());
  }

  public AngularVelocity getPivotVelocity() {
    return rightPivotMotorLeader.getRotorVelocity().getValue();
  }

  public AngularVelocity getWristVelocity() {
    return wristPivotMotor.getRotorVelocity().getValue();
  }

  public AngularVelocity getLiftVelocity() {
    return rightLiftMotorLeader.getRotorVelocity().getValue();
  }

  public boolean isLiftVelocityZero() {
    return getLiftVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
  }

  public boolean isPivotVelocityZero() {
    return getPivotVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
  }

  public boolean isWristVelocityZero() {
    return getWristVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
  }

  public void resetLiftSensorPosition(Distance setpoint) {
    rightLiftMotorLeader.setPosition(setpoint.in(Inches));
    leftLiftMotorFollower.setPosition(setpoint.in(Inches));
  }

  public void resetPivotSensorPosition(Angle setpoint) {
    rightPivotMotorLeader.setPosition(setpoint.in(Degrees));
    leftPivotMotorFollower.setPosition(setpoint.in(Degrees));
  }

  public void resetWristSensorPosition(Angle setpoint) {
    wristPivotMotor.setPosition(setpoint.in(Degrees));
  }

  public boolean arePositionsAtSetPoint(MechanismPositionGroup positionGroup) {
    return (getLiftPosition().compareTo(positionGroup.liftHeight.minus(positionGroup.liftTolerance)) > 0 &&
        getLiftPosition().compareTo(positionGroup.liftHeight.plus(positionGroup.liftTolerance)) < 0 &&
        getPivotAngle().compareTo(positionGroup.pivotAngle.minus(positionGroup.pivotTolerance)) > 0 &&
        getPivotAngle().compareTo(positionGroup.pivotAngle.plus(positionGroup.pivotTolerance)) < 0 &&
        getWristAngle().compareTo(positionGroup.wristAngle.minus(positionGroup.wristTolerance)) > 0 &&
        getWristAngle().compareTo(positionGroup.wristAngle.plus(positionGroup.wristTolerance)) < 0);
  }

  @Override
  public void periodic() {
    //

  }
}
