// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;

import java.lang.Thread.State;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constDrivetrain;
import frc.robot.Constants.constField;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.StateMachine.DriverState;

public class DriveManual extends Command {
  Drivetrain subDrivetrain;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  boolean isOpenLoop;
  double redAllianceMultiplier = 1;
  StateMachine subStateMachine;

  public DriveManual(Drivetrain subDrivetrain, StateMachine subStateMachine, DoubleSupplier xAxis, DoubleSupplier yAxis,
      DoubleSupplier rotationAxis) {
    this.subDrivetrain = subDrivetrain;
    this.subStateMachine = subStateMachine;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;

    isOpenLoop = true;

    addRequirements(this.subDrivetrain);
    addRequirements(this.subStateMachine);
  }

  @Override
  public void initialize() {
    redAllianceMultiplier = constField.isRedAlliance() ? -1 : 1;
  }

  @Override
  public void execute() {
    // Get Joystick inputs
    double xVelocity = xAxis.getAsDouble() * constDrivetrain.REAL_DRIVE_SPEED.in(Units.MetersPerSecond)
        * redAllianceMultiplier;
    double yVelocity = -yAxis.getAsDouble() * constDrivetrain.REAL_DRIVE_SPEED.in(Units.MetersPerSecond)
        * redAllianceMultiplier;
    double rVelocity = -rotationAxis.getAsDouble() * constDrivetrain.TURN_SPEED.in(Units.RadiansPerSecond);

    subStateMachine.setDriverState(StateMachine.DriverState.MANUAL);

    subDrivetrain.drive(
        new Translation2d(xVelocity, yVelocity), rVelocity, isOpenLoop);

  }

  @Override
  public void end(boolean interrupted) {
    subDrivetrain.neutralDriveOutputs();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
