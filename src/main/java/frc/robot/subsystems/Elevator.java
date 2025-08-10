// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Degrees;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import edu.wpi.first.units.Units;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Constants.constElevator;
// import frc.robot.Robot;
// import frc.robot.RobotMap.mapElevator;

// public class Elevator extends SubsystemBase {
// TalonFX leftLiftMotorFollower;
// TalonFX rightLiftMotorLeader;
// TalonFX leftPivotMotorFollower;
// TalonFX rightPivotMotorLeader;

// private Angle lastDesiredAngle = Degrees.zero();

// private Distance lastDesiredPosition;
// MotionMagicExpoVoltage positionRequest = new MotionMagicExpoVoltage(0);

// /** Creates a new Elevator. */
// public Elevator() {

// leftLiftMotorFollower = new TalonFX(mapElevator.LEFT_LIFT_CAN); // Elevator
// left motor
// rightLiftMotorLeader = new TalonFX(mapElevator.RIGHT_LIFT_CAN); // Elevator
// right motor
// leftPivotMotorFollower = new TalonFX(mapElevator.LEFT_PIVOT_CAN); // Elevator
// left pivot motor
// rightPivotMotorLeader = new TalonFX(mapElevator.RIGHT_PIVOT_CAN); // Elevator
// right pivot motor

// lastDesiredPosition = Units.Inches.of(0);
// // Set default motor configurations if needed
// // e.g., elevatorLeftMotor.configFactoryDefault();
// leftLiftMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_LIFT_CONFIG);
// rightLiftMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_LIFT_CONFIG);
// leftPivotMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_PIVOT_CONFIG);
// rightPivotMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_PIVOT_CONFIG);
// }

// public AngularVelocity getMotorVelocity() {
// return rightLiftMotorLeader.getRotorVelocity().getValue();
// }

// public boolean isMotorVelocityZero() {
// return getMotorVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
// }

// public Distance getLastDesiredLiftPosition() {// elevator extension
// return lastDesiredPosition;
// }

// public Distance getLiftPosition() {
// if (Robot.isSimulation()) {
// return getLastDesiredLiftPosition();
// }
// return
// Units.Inches.of(rightLiftMotorLeader.getPosition().getValueAsDouble());
// }

// public void setLiftPosition(Distance height) {
// rightLiftMotorLeader.setControl(positionRequest.withPosition(height.in(Units.Inches)));
// leftLiftMotorFollower.setControl(new
// Follower(rightLiftMotorLeader.getDeviceID(), true));
// lastDesiredPosition = height;
// }

// public boolean isLiftAtSpecificSetpoint(Distance setpoint) {
// return isLiftAtSetPointWithTolerance(setpoint,
// Constants.constElevator.DEADZONE_DISTANCE);
// }

// public boolean isLiftAtSetPointWithTolerance(Distance position, Distance
// tolerance) {
// if (Robot.isSimulation()) {
// return true;
// }
// return (getLiftPosition()
// .compareTo(position.minus(tolerance)) > 0) &&
// getLiftPosition().compareTo(position.plus(tolerance)) < 0;
// }

// public Angle getElevatorPivotAngle() {
// return rightPivotMotorLeader.getPosition().getValue();
// }

// public Angle getLastDesiredPivotAngle() {
// return lastDesiredAngle;
// }

// public void setElevatorPivotAngle(Angle angle) {
// rightPivotMotorLeader.setControl(positionRequest.withPosition(angle.in(Degrees)));
// leftPivotMotorFollower.setControl(new
// Follower(rightPivotMotorLeader.getDeviceID(), true));
// lastDesiredAngle = angle;
// }

// @Override
// public void periodic() {
// // This method will be called once per scheduler run
// }
// }
