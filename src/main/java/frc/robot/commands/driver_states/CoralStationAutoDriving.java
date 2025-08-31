// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driver_states;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.Field;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralStationAutoDriving extends Command {
  /** Creates a new CoralStationAutoDriving. */
  double redAllianceMultiplier = 1;
  boolean isRedAlliance = Field.isRedAlliance();
  Drivetrain subDrivetrain;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  DriverStateMachine subDriverStateMachine;
  boolean isOpenLoop;
  boolean farCoralStation;

  public CoralStationAutoDriving(Drivetrain subDrivetrain, DriverStateMachine subDriverStateMachine,
      DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier rotationAxis, boolean farCoralStation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subDrivetrain = subDrivetrain;
    this.subDriverStateMachine = subDriverStateMachine;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    addRequirements(this.subDrivetrain);
    addRequirements(this.subDriverStateMachine);
    isOpenLoop = true;
    this.farCoralStation = farCoralStation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    redAllianceMultiplier = Field.isRedAlliance() ? -1 : 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isInAutoDriveZone = subDrivetrain.isInAutoDriveZone(
        Field.CORAL_STATION_AUTO_DRIVE_MAX_DISTANCE,
        Field.getCoralStationPositions().get());
    LinearVelocity xVelocity = Units.MetersPerSecond
        .of(xAxis.getAsDouble() * constDrivetrain.REAL_DRIVE_SPEED.in(Units.MetersPerSecond) * redAllianceMultiplier);
    LinearVelocity yVelocity = Units.MetersPerSecond
        .of(-yAxis.getAsDouble() * constDrivetrain.REAL_DRIVE_SPEED.in(Units.MetersPerSecond) * redAllianceMultiplier);

    if (isInAutoDriveZone) {
      Pose2d closestPose;
      if (farCoralStation) {
        closestPose = subDrivetrain
            .getDesiredPose(List.of(Field.getCoralStationPositions().get().get(0),
                Field.getCoralStationPositions().get().get(2), Field.getCoralStationPositions().get().get(4),
                Field.getCoralStationPositions().get().get(6)));
        subDriverStateMachine.setDriverState(DriverStateMachine.DriverState.CORAL_STATION_AUTO_DRIVING_FAR);
      } else {
        closestPose = subDrivetrain
            .getDesiredPose(List.of(Field.getCoralStationPositions().get().get(1),
                Field.getCoralStationPositions().get().get(3), Field.getCoralStationPositions().get().get(5),
                Field.getCoralStationPositions().get().get(7)));
        subDriverStateMachine.setDriverState(DriverStateMachine.DriverState.CORAL_STATION_AUTO_DRIVING_CLOSE);
      }
      subDrivetrain.autoAlign(isRedAlliance,
          closestPose,
          xVelocity,
          yVelocity,
          isOpenLoop,
          false,
          false);
    } else {
      subDrivetrain.rotationalAlign(isRedAlliance,
          subDrivetrain.getDesiredPose(Field.getCoralStationPositions().get()),
          xVelocity,
          yVelocity,
          isOpenLoop);
      subDriverStateMachine.setDriverState(DriverStateMachine.DriverState.CORAL_STATION_ROTATION_SNAPPING);
      System.out
          .println("Auto-drive zone check failed: Robot is not within the required auto-drive zone. Current position: "
              + subDrivetrain.getPose() + ". Expected zone: " + Field.getReefPositions().get()
              + "What is your problem????????????");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
