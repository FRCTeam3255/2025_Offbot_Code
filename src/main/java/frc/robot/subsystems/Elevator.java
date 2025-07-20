// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapElevator;

public class Elevator extends SubsystemBase {
  TalonFX elevatorLeftMotor;
  TalonFX elevatorRightMotor;
  TalonFX elevatorLeftPivotMotor;
  TalonFX elevatorRightPivotMotor;

  /** Creates a new Elevator. */
  public Elevator() {

    elevatorLeftMotor = new TalonFX(mapElevator.ELEVATOR_LEFT_CAN); // Elevator left motor
    elevatorRightMotor = new TalonFX(mapElevator.ELEVATOR_RIGHT_CAN); // Elevator right motor
    elevatorLeftPivotMotor = new TalonFX(mapElevator.ELEVATOR_LEFT_PIVOT_CAN); // Elevator left pivot motor
    elevatorRightPivotMotor = new TalonFX(mapElevator.ELEVATOR_RIGHT_PIVOT_CAN); // Elevator right pivot motor

    // Set default motor configurations if needed
    // e.g., elevatorLeftMotor.configFactoryDefault();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
