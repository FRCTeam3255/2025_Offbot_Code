// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MechanismPositionGroup;
import frc.robot.Constants.constMotion;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.*;

public class Motion extends SubsystemBase {
  /** Creates a new Motion. */
  TalonFX leftLiftMotorFollower;
  TalonFX rightLiftMotorLeader;
  TalonFX leftPivotMotorFollower;
  TalonFX rightPivotMotorLeader;
  TalonFX intakePivotMotor;

  private Angle elevatorPivotLastDesiredAngle = Degrees.zero();
  private Angle intakeWristLastDesiredAngle = Degrees.zero();

  private Distance elevatorLiftLastDesiredPosition;
  MotionMagicExpoVoltage positionRequest = new MotionMagicExpoVoltage(0);

  public Motion() {
    leftLiftMotorFollower = new TalonFX(mapMotion.LEFT_LIFT_CAN); // Elevator left motor
    rightLiftMotorLeader = new TalonFX(mapMotion.RIGHT_LIFT_CAN); // Elevator right motor
    leftPivotMotorFollower = new TalonFX(mapMotion.LEFT_PIVOT_CAN); // Elevator left pivot motor
    rightPivotMotorLeader = new TalonFX(mapMotion.RIGHT_PIVOT_CAN); // Elevator right pivot motor
    intakePivotMotor = new TalonFX(mapMotion.INTAKE_PIVOT_CAN); // Intake pivot motor

    elevatorLiftLastDesiredPosition = Units.Inches.of(0);
    // Set default motor configurations if needed
    // e.g., elevatorLeftMotor.configFactoryDefault();
    leftLiftMotorFollower.getConfigurator().apply(constMotion.LIFT_CONFIG);
    rightLiftMotorLeader.getConfigurator().apply(constMotion.LIFT_CONFIG);
    leftPivotMotorFollower.getConfigurator().apply(constMotion.ELEVATOR_PIVOT_CONFIG);
    rightPivotMotorLeader.getConfigurator().apply(constMotion.ELEVATOR_PIVOT_CONFIG);
    intakePivotMotor.getConfigurator().apply(constMotion.WRIST_CONFIG);
  }

  public AngularVelocity getLiftMotorVelocity() {
    return rightLiftMotorLeader.getRotorVelocity().getValue();
  }

  public boolean isLiftMotorVelocityZero() {
    return getLiftMotorVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
  }

  public Distance getLastDesiredLiftPosition() {// elevator extension
    return elevatorLiftLastDesiredPosition;
  }

  public Distance getLiftPosition() {
    if (Robot.isSimulation()) {
      return getLastDesiredLiftPosition();
    }
    return Units.Inches.of(rightLiftMotorLeader.getPosition().getValueAsDouble());
  }

  public void setLiftPosition(Distance height) {
    rightLiftMotorLeader.setControl(positionRequest.withPosition(height.in(Units.Inches)));
    leftLiftMotorFollower.setControl(new Follower(rightLiftMotorLeader.getDeviceID(), true));
    elevatorLiftLastDesiredPosition = height;
  }

  public boolean isLiftAtSpecificSetpoint(Distance setpoint) {
    return isLiftAtSetPointWithTolerance(setpoint, Constants.constMotion.DEADZONE_DISTANCE);
  }

  public boolean isLiftAtSetPointWithTolerance(Distance position, Distance tolerance) {
    if (Robot.isSimulation()) {
      return true;
    }
    return (getLiftPosition()
        .compareTo(position.minus(tolerance)) > 0) &&
        getLiftPosition().compareTo(position.plus(tolerance)) < 0;
  }

  public Angle getElevatorPivotAngle() {
    return rightPivotMotorLeader.getPosition().getValue();
  }

  public Angle getLastDesiredPivotAngle() {
    return elevatorPivotLastDesiredAngle;
  }

  public void setElevatorPivotAngle(Angle angle) {
    rightPivotMotorLeader.setControl(positionRequest.withPosition(angle.in(Degrees)));
    leftPivotMotorFollower.setControl(new Follower(rightPivotMotorLeader.getDeviceID(), true));
    elevatorPivotLastDesiredAngle = angle;
  }

  public void getWristPivotMotorAngle() {
    double angle = intakePivotMotor.getPosition().getValueAsDouble();

  }

  public Angle getLastDesiredWristPivotAngle() {
    return intakeWristLastDesiredAngle;
  }

  public void setWristPivotAngle(Angle angle) {
    intakePivotMotor.setControl(positionRequest.withPosition(angle.in(Degrees)));
    intakeWristLastDesiredAngle = angle;
  }

  public void setAllPosition(MechanismPositionGroup positionGroup) {
    setLiftPosition(positionGroup.liftHeight);
    setElevatorPivotAngle(positionGroup.pivotAngle);
    setWristPivotAngle(positionGroup.wristAngle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
