// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.constElevator;
import frc.robot.Robot;
import frc.robot.RobotMap.mapElevator;

public class Elevator extends SubsystemBase {
  TalonFX leftMotorFollower;
  TalonFX rightMotorLeader;
  TalonFX elevatorLeftPivotMotor;
  TalonFX elevatorRightPivotMotor;

  private Angle lastDesiredAngle = Degrees.zero();

  Distance currentLeftPosition = Units.Inches.of(0);
  Distance currentRightPosition = Units.Inches.of(0);
  private Distance lastDesiredPosition;
  MotionMagicExpoVoltage positionRequest = new MotionMagicExpoVoltage(0);

  /** Creates a new Elevator. */
  public Elevator() {

    leftMotorFollower = new TalonFX(mapElevator.ELEVATOR_LEFT_CAN); // Elevator left motor
    rightMotorLeader = new TalonFX(mapElevator.ELEVATOR_RIGHT_CAN); // Elevator right motor
    elevatorLeftPivotMotor = new TalonFX(mapElevator.ELEVATOR_LEFT_PIVOT_CAN); // Elevator left pivot motor
    elevatorRightPivotMotor = new TalonFX(mapElevator.ELEVATOR_RIGHT_PIVOT_CAN); // Elevator right pivot motor

    lastDesiredPosition = Units.Inches.of(0);
    // Set default motor configurations if needed
    // e.g., elevatorLeftMotor.configFactoryDefault();
    leftMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_MOTOR_CONFIG);
    rightMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_MOTOR_CONFIG);
    elevatorLeftPivotMotor.getConfigurator().apply(constElevator.ELEVATOR_PIVOT_CONFIG);
    elevatorRightPivotMotor.getConfigurator().apply(constElevator.ELEVATOR_PIVOT_CONFIG);
  }

  public AngularVelocity getRotorVelocity() {
    return rightMotorLeader.getRotorVelocity().getValue();
  }

  public boolean isRotorVelocityZero() {
    return getRotorVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
  }

  public Distance getLastDesiredPosition() {
    return lastDesiredPosition;
  }

  public Distance getElevatorPosition() {
    if (Robot.isSimulation()) {
      return getLastDesiredPosition();
    }
    return Units.Inches.of(rightMotorLeader.getPosition().getValueAsDouble());
  }

  public double getRightPosition(Distance height) {
    return rightMotorLeader.getPosition().getValueAsDouble();
  }

  public double getLeftPosition(Distance height) {
    return leftMotorFollower.getPosition().getValueAsDouble();
  }

  public void setElevatorPosition(Distance height) {
    rightMotorLeader.setControl(positionRequest.withPosition(height.in(Units.Inches)));
    leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), true));
    lastDesiredPosition = height;
  }

  public boolean isAtSpecificSetpoint(Distance setpoint) {
    return isAtSetPointWithTolerance(setpoint, Constants.constElevator.DEADZONE_DISTANCE);
  }

  public boolean isAtAnyCoralScoringPosition() {
    if (isAtSpecificSetpoint(constElevator.CORAL_L1_HEIGHT) ||
        isAtSpecificSetpoint(constElevator.CORAL_L2_HEIGHT) ||
        isAtSpecificSetpoint(constElevator.CORAL_L3_HEIGHT) ||
        isAtSpecificSetpoint(constElevator.CORAL_L4_HEIGHT)) {
      return true;
    }
    return false;
  }

  public boolean isAtAnyAlgaeScoringPosition() {
    if (isAtSpecificSetpoint(constElevator.ALGAE_NET_HEIGHT)) {
      return true;
    }
    return false;
  }

  public boolean isAtSetPointWithTolerance(Distance position, Distance tolerance) {
    if (Robot.isSimulation()) {
      return true;
    }
    return (getElevatorPosition()
        .compareTo(position.minus(tolerance)) > 0) &&
        getElevatorPosition().compareTo(position.plus(tolerance)) < 0;
  }

  public Angle getPivotAngle() {
    return elevatorRightPivotMotor.getPosition().getValue();
  }

  public Angle getLeftPivotAngle() {
    return elevatorLeftPivotMotor.getPosition().getValue();
  }

  public Angle getLastDesiredPivotAngle() {
    return lastDesiredAngle;
  }

  public void setPivotAngle(Angle angle) {
    elevatorRightPivotMotor.setControl(positionRequest.withPosition(angle.in(Degrees)));
    elevatorLeftPivotMotor.setControl(new Follower(elevatorRightPivotMotor.getDeviceID(), true));
    lastDesiredAngle = angle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
