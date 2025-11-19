// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driver_states;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constDrivetrain;
import frc.robot.subsystems.DriverStateMachine;
import frc.robot.subsystems.Drivetrain;

public class DriveManual extends Command {
  Drivetrain subDrivetrain;
  DoubleSupplier xAxis, yAxis, rotationXAxis, rotationYAxis;
  boolean isOpenLoop;
  DriverStateMachine subDriverStateMachine;
  BooleanSupplier slowMode;

  public DriveManual(Drivetrain subDrivetrain, DriverStateMachine subDriverStateMachine, DoubleSupplier xAxis,
      DoubleSupplier yAxis, DoubleSupplier rotationXAxis, DoubleSupplier rotationYAxis, BooleanSupplier slowMode) {
    this.subDrivetrain = subDrivetrain;
    this.subDriverStateMachine = subDriverStateMachine;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.slowMode = slowMode;
    this.rotationXAxis = rotationXAxis;
    this.rotationYAxis = rotationYAxis;

    isOpenLoop = true;

    addRequirements(this.subDrivetrain);
    addRequirements(this.subDriverStateMachine);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    var velocities = subDrivetrain.calculateVelocitiesFromManualInput(xAxis, yAxis, rotationXAxis, rotationYAxis,
        slowMode);

    subDriverStateMachine.setDriverState(DriverStateMachine.DriverState.MANUAL);

    if (constDrivetrain.INVERT_ROTATION) {
      velocities.omegaRadiansPerSecond = -velocities.omegaRadiansPerSecond;
    }

    subDrivetrain.drive(
        new Translation2d(velocities.vxMetersPerSecond, velocities.vyMetersPerSecond),
        velocities.omegaRadiansPerSecond, isOpenLoop);

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