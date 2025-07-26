// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constClimber;
import frc.robot.RobotMap.mapClimber;

public class Climber extends SubsystemBase {
  TalonFX cageCollectMotor;

  /** Creates a new Climber. */
  public Climber() {
    cageCollectMotor = new TalonFX(mapClimber.CAGE_COLLECTER_CAN);

    cageCollectMotor.getConfigurator().apply(constClimber.CLIMBER_CONFIG);

  }

  public void setClimberMotorPercentOutput(double speed) {
    cageCollectMotor.set(speed);
  }

  public void setClimberNuetralOutput() {
    cageCollectMotor.setControl(new NeutralOut());
  }

  @Override
  public void periodic() {

  }
}
