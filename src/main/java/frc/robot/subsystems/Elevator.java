// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.mapElevator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemBase {
  TalonFX elevatorLeftMotor;
  TalonFX elevatorRightMotor;
  TalonFX elevatorLeftPivotMotor;
  TalonFX elevatorRightPivotMotor;
  MechanismLigament2d elevator;

  /** Creates a new Elevator. */
  public Elevator() {

    elevatorLeftMotor = new TalonFX(mapElevator.ELEVATOR_LEFT_CAN); // Elevator left motor
    elevatorRightMotor = new TalonFX(mapElevator.ELEVATOR_RIGHT_CAN); // Elevator right motor
    elevatorLeftPivotMotor = new TalonFX(mapElevator.ELEVATOR_LEFT_PIVOT_CAN); // Elevator left pivot motor
    elevatorRightPivotMotor = new TalonFX(mapElevator.ELEVATOR_RIGHT_PIVOT_CAN); // Elevator right pivot motor

    // the main mechanism object
    Mechanism2d mech = new Mechanism2d(3, 3);
    // the mechanism root node
    MechanismRoot2d root = mech.getRoot("climber", 2, 0);
    elevator = root.append(new MechanismLigament2d("elevator", .5, 90));
    // Set default motor configurations if needed
    // e.g., elevatorLeftMotor.configFactoryDefault();
    // post the mechanism to the dashboard
    SmartDashboard.putData("Mech2d", mech);
  }

  public void setAngle(Angle agnle) {
    elevatorLeftPivotMotor.setPosition(agnle);

  }

  public Angle getAngle() {
    return elevatorLeftPivotMotor.getPosition().getValue();
  }

  @Override
  public void periodic() {
    elevator.setLength(2);
    // This method will be called once per scheduler run
  }
}
