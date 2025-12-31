// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driver_states;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriverState;
import frc.robot.RobotContainer;

public class DriveManual extends Command {
  boolean isOpenLoop;

  public DriveManual() {
    isOpenLoop = true;
    addRequirements(RobotContainer.subDrivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    var velocities = RobotContainer.subDrivetrain.calculateVelocitiesFromInput(
        RobotContainer.conDriver.axis_LeftY,
        RobotContainer.conDriver.axis_LeftX,
        RobotContainer.conDriver.axis_RightX,
        RobotContainer.conDriver.btn_RightBumper);

    RobotContainer.setDriverState(DriverState.MANUAL);

    RobotContainer.subDrivetrain.drive(
        new Translation2d(velocities.vxMetersPerSecond, velocities.vyMetersPerSecond),
        velocities.omegaRadiansPerSecond,
        isOpenLoop);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.subDrivetrain.neutralDriveOutputs();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
