// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driver_states;

import frc.robot.subsystems.*;
import java.lang.Thread.State;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.DriverStateMachine;
import frc.robot.subsystems.Drivetrain;

public class DriveManual extends Command {
  Drivetrain subDrivetrain;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  boolean isOpenLoop;
  DriverStateMachine subDriverStateMachine;
  Motion globalMotion;

  public DriveManual(Drivetrain subDrivetrain, DriverStateMachine subDriverStateMachine, DoubleSupplier xAxis,
      DoubleSupplier yAxis, DoubleSupplier rotationAxis, Motion subMotion) {
    this.subDrivetrain = subDrivetrain;
    this.subDriverStateMachine = subDriverStateMachine;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    globalMotion = subMotion;

    isOpenLoop = true;

    addRequirements(this.subDrivetrain);
    addRequirements(this.subDriverStateMachine);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    var velocities = subDrivetrain.calculateVelocitiesFromInput(xAxis, yAxis, rotationAxis)
                - globalMotion.getLiftPosition().in(Units.Meters) / constMotion.HEIGHT_DIVIDER.in(Units.Meters);

    subDriverStateMachine.setDriverState(DriverStateMachine.DriverState.MANUAL);

    subDrivetrain.drive(
        new Translation2d(velocities.x, velocities.y),
        velocities.rotation,
        isOpenLoop);
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
