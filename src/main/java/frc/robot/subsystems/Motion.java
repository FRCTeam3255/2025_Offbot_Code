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
  TalonFX backLeftPivotMotorFollower;
  TalonFX backRightPivotMotorFollower;
  TalonFX frontLeftPivotMotorFollower;
  TalonFX frontRightPivotMotorLeader;
  TalonFX wristPivotMotor;

  private Angle elevatorPivotLastDesiredAngle = Degrees.zero();
  private Angle wristLastDesiredAngle = Degrees.zero();
  private Distance elevatorLiftLastDesiredPosition = Units.Inches.zero();
  MotionMagicExpoVoltage positionRequest = new MotionMagicExpoVoltage(0);
  public boolean attemptingLiftZeroing = false;
  public boolean attemptingPivotZeroing = false;
  public boolean attemptingWristZeroing = false;
  public boolean hasLiftZeroed = false;
  public boolean hasPivotZeroed = false;
  public boolean hasWristZeroed = false;
  public boolean hasSetStartingConfig = false;

  public Motion() {
    leftLiftMotorFollower = new TalonFX(mapMotion.LEFT_LIFT_CAN);
    rightLiftMotorLeader = new TalonFX(mapMotion.RIGHT_LIFT_CAN);
    backLeftPivotMotorFollower = new TalonFX(mapMotion.BACK_LEFT_PIVOT_CAN);
    backRightPivotMotorFollower = new TalonFX(mapMotion.BACK_RIGHT_PIVOT_CAN);
    frontLeftPivotMotorFollower = new TalonFX(mapMotion.FRONT_LEFT_PIVOT_CAN);
    frontRightPivotMotorLeader = new TalonFX(mapMotion.FRONT_RIGHT_PIVOT_CAN);
    wristPivotMotor = new TalonFX(mapMotion.INTAKE_PIVOT_CAN);

    elevatorLiftLastDesiredPosition = Units.Inches.of(0);
    // Set default motor configurations if needed
    // e.g., elevatorLeftMotor.configFactoryDefault();
    leftLiftMotorFollower.getConfigurator().apply(constMotion.LIFT_CONFIG);
    rightLiftMotorLeader.getConfigurator().apply(constMotion.LIFT_CONFIG);
    backLeftPivotMotorFollower.getConfigurator().apply(constMotion.ELEVATOR_PIVOT_CONFIG);
    backRightPivotMotorFollower.getConfigurator().apply(constMotion.ELEVATOR_PIVOT_CONFIG);
    frontLeftPivotMotorFollower.getConfigurator().apply(constMotion.ELEVATOR_PIVOT_CONFIG);
    frontRightPivotMotorLeader.getConfigurator().apply(constMotion.ELEVATOR_PIVOT_CONFIG);
    wristPivotMotor.getConfigurator().apply(constMotion.WRIST_CONFIG);
  }

  private void setLiftPosition(Distance height, int slot) {
    rightLiftMotorLeader.setControl(positionRequest.withPosition(height.in(Units.Inches)).withSlot(slot));
    leftLiftMotorFollower.setControl(new Follower(rightLiftMotorLeader.getDeviceID(), true));
    elevatorLiftLastDesiredPosition = height;
  }

  private void setElevatorPivotAngle(Angle angle, int slot) {
    frontRightPivotMotorLeader.setControl(positionRequest.withPosition(angle).withSlot(slot));
    frontLeftPivotMotorFollower.setControl(new Follower(frontRightPivotMotorLeader.getDeviceID(), true));
    backLeftPivotMotorFollower.setControl(new Follower(frontRightPivotMotorLeader.getDeviceID(), true));
    backRightPivotMotorFollower.setControl(new Follower(frontRightPivotMotorLeader.getDeviceID(), false));
    elevatorPivotLastDesiredAngle = angle;
  }

  private void setWristPivotAngle(Angle angle, int slot) {
    wristPivotMotor.setControl(positionRequest.withPosition(angle).withSlot(slot));
    wristLastDesiredAngle = angle;
  }

  public void setAllPosition(MechanismPositionGroup positionGroup) {
    if (isWristInDanger() == true) {
      setWristPivotAngle(constMotion.WRIST_DANGER_ANGLE, positionGroup.wristSlot);
    } else {
      setWristPivotAngle(positionGroup.wristAngle, positionGroup.wristSlot);
    }
    setElevatorPivotAngle(positionGroup.pivotAngle, positionGroup.pivotSlot);
    setLiftPosition(positionGroup.liftHeight, positionGroup.liftSlot);
  }

  public void setLiftCoastMode(boolean coastMode) {
    if (coastMode) {
      constMotion.LIFT_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      rightLiftMotorLeader.getConfigurator().apply(constMotion.LIFT_CONFIG);
      leftLiftMotorFollower.getConfigurator().apply(constMotion.LIFT_CONFIG);
    } else {
      constMotion.LIFT_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      rightLiftMotorLeader.getConfigurator().apply(constMotion.LIFT_CONFIG);
      leftLiftMotorFollower.getConfigurator().apply(constMotion.LIFT_CONFIG);
    }
  }

  public void setPivotCoastMode(boolean coastMode) {
    if (coastMode) {
      constMotion.ELEVATOR_PIVOT_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      frontRightPivotMotorLeader.getConfigurator().apply(constMotion.ELEVATOR_PIVOT_CONFIG);
      frontLeftPivotMotorFollower.getConfigurator().apply(constMotion.ELEVATOR_PIVOT_CONFIG);
      backRightPivotMotorFollower.getConfigurator().apply(constMotion.ELEVATOR_PIVOT_CONFIG);
      backLeftPivotMotorFollower.getConfigurator().apply(constMotion.ELEVATOR_PIVOT_CONFIG);
    } else {
      constMotion.ELEVATOR_PIVOT_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      frontRightPivotMotorLeader.getConfigurator().apply(constMotion.ELEVATOR_PIVOT_CONFIG);
      frontLeftPivotMotorFollower.getConfigurator().apply(constMotion.ELEVATOR_PIVOT_CONFIG);
      backRightPivotMotorFollower.getConfigurator().apply(constMotion.ELEVATOR_PIVOT_CONFIG);
      backLeftPivotMotorFollower.getConfigurator().apply(constMotion.ELEVATOR_PIVOT_CONFIG);
    }
  }

  public void setWristCoastMode(boolean coastMode) {
    if (coastMode) {
      constMotion.WRIST_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      wristPivotMotor.getConfigurator().apply(constMotion.WRIST_CONFIG);
    } else {
      constMotion.WRIST_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      wristPivotMotor.getConfigurator().apply(constMotion.WRIST_CONFIG);
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
    return frontRightPivotMotorLeader.getPosition().getValue();
  }

  public Angle getWristAngle() {
    if (Robot.isSimulation()) {
      return wristLastDesiredAngle;
    }
    return wristPivotMotor.getPosition().getValue();
  }

  public AngularVelocity getPivotVelocity() {
    return frontRightPivotMotorLeader.getRotorVelocity().getValue();
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

  public boolean isWristInDanger() {
    return getWristAngle().lt(constMotion.WRIST_DANGER_ANGLE)
        && elevatorPivotLastDesiredAngle.lt(constMotion.PIVOT_DANGER_ANGLE);
  }

  public void resetLiftSensorPosition(Distance setpoint) {
    rightLiftMotorLeader.setPosition(setpoint.in(Inches));
    leftLiftMotorFollower.setPosition(setpoint.in(Inches));
  }

  public void resetPivotSensorPosition(Angle setpoint) {
    frontRightPivotMotorLeader.setPosition(setpoint.in(Degrees));
    frontLeftPivotMotorFollower.setPosition(setpoint.in(Degrees));
    backRightPivotMotorFollower.setPosition(setpoint.in(Degrees));
    backLeftPivotMotorFollower.setPosition(setpoint.in(Degrees));
  }

  public void resetWristSensorPosition(Angle setpoint) {
    wristPivotMotor.setPosition(setpoint);
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
