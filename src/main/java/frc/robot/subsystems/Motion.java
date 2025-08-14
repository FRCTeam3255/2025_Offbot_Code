// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
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
  TalonFX intakePivotMotor;

  private Angle elevatorPivotLastDesiredAngle = Degrees.zero();
  private Angle intakeWristLastDesiredAngle = Degrees.zero();
  private Distance elevatorLiftLastDesiredPosition = Units.Inches.zero();
  MotionMagicExpoVoltage positionRequest = new MotionMagicExpoVoltage(0);

  public Motion() {
    leftLiftMotorFollower = new TalonFX(mapMotion.LEFT_LIFT_CAN);
    rightLiftMotorLeader = new TalonFX(mapMotion.RIGHT_LIFT_CAN);
    leftPivotMotorFollower = new TalonFX(mapMotion.LEFT_PIVOT_CAN);
    rightPivotMotorLeader = new TalonFX(mapMotion.RIGHT_PIVOT_CAN);
    intakePivotMotor = new TalonFX(mapMotion.INTAKE_PIVOT_CAN);

    elevatorLiftLastDesiredPosition = Units.Inches.of(0);
    // Set default motor configurations if needed
    // e.g., elevatorLeftMotor.configFactoryDefault();
    leftLiftMotorFollower.getConfigurator().apply(constMotion.LIFT_CONFIG);
    rightLiftMotorLeader.getConfigurator().apply(constMotion.LIFT_CONFIG);
    leftPivotMotorFollower.getConfigurator().apply(constMotion.ELEVATOR_PIVOT_CONFIG);
    rightPivotMotorLeader.getConfigurator().apply(constMotion.ELEVATOR_PIVOT_CONFIG);
    intakePivotMotor.getConfigurator().apply(constMotion.WRIST_CONFIG);
  }

  public Distance getLiftPosition() {
    if (Robot.isSimulation()) {
      return elevatorLiftLastDesiredPosition;
    }
    return Units.Inches.of(rightLiftMotorLeader.getPosition().getValueAsDouble());
  }

  private void setLiftPosition(Distance height) {
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
    intakePivotMotor.setControl(positionRequest.withPosition(angle.in(Degrees)));
    intakeWristLastDesiredAngle = angle;
  }

  public void setAllPosition(MechanismPositionGroup positionGroup) {
    setLiftPosition(positionGroup.liftHeight);
    setElevatorPivotAngle(positionGroup.pivotAngle);
    setWristPivotAngle(positionGroup.wristAngle);
  }

  public Angle getPivotAngle() {
    if (Robot.isSimulation()) {
      return elevatorPivotLastDesiredAngle;
    }
    return rightPivotMotorLeader.getPosition().getValue();
  }

  public Angle getWristAngle() {
    if (Robot.isSimulation()) {
      return intakeWristLastDesiredAngle;
    }
    return Degrees.of(intakePivotMotor.getPosition().getValueAsDouble());
  }

  public boolean arePositionsAtSetPoint(Distance liftTolerance, Angle pivotTolerance, Angle wristTolerance) {
    if (Robot.isSimulation()) {
      return true;
    }
    return (getLiftPosition().compareTo(elevatorLiftLastDesiredPosition.minus(liftTolerance)) > 0 &&
        getLiftPosition().compareTo(elevatorLiftLastDesiredPosition.plus(liftTolerance)) < 0 &&
        getPivotAngle().compareTo(elevatorPivotLastDesiredAngle.minus(pivotTolerance)) > 0 &&
        getPivotAngle().compareTo(elevatorPivotLastDesiredAngle.plus(pivotTolerance)) < 0 &&
        getWristAngle().compareTo(intakeWristLastDesiredAngle.minus(wristTolerance)) > 0 &&
        getWristAngle().compareTo(intakeWristLastDesiredAngle.plus(wristTolerance)) < 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
