// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driver_states;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.DriverStateMachine;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Field;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefAutoDriving extends Command {
  /** Creates a new ReefAutoDriving. */
  double redAllianceMultiplier = 1;
  boolean isRedAlliance = Field.isRedAlliance();
  Drivetrain subDrivetrain;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  DriverStateMachine subDriverStateMachine;
  boolean isOpenLoop;
  boolean leftBranch;
  Pose2d closestPoseByRotation;
  Pose2d getLeftPos;
  Pose2d getRightPos;
  Pose2d getAllPos;

  public ReefAutoDriving(Drivetrain subDrivetrain, DriverStateMachine subDriverStateMachine,
      DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier rotationAxis, boolean leftBranch) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subDrivetrain = subDrivetrain;
    this.subDriverStateMachine = subDriverStateMachine;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    addRequirements(this.subDrivetrain);
    addRequirements(this.subDriverStateMachine);
    isOpenLoop = true;
    this.leftBranch = leftBranch;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftBranch = leftBranch;
    redAllianceMultiplier = Field.isRedAlliance() ? -1 : 1;
    getLeftPos = subDrivetrain
        .getDesiredPose(subDrivetrain.autoDrivePositions(Field.getLeftReefPositions(isRedAlliance).get()).get());
    getRightPos = subDrivetrain
        .getDesiredPose(subDrivetrain.autoDrivePositions(Field.getRightReefPositions(isRedAlliance).get()).get());
    getAllPos = subDrivetrain
        .getDesiredPose(subDrivetrain.autoDrivePositions(Field.getReefPositions(isRedAlliance).get()).get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LinearVelocity xVelocity = Units.MetersPerSecond
        .of(xAxis.getAsDouble() * constDrivetrain.REAL_DRIVE_SPEED.in(Units.MetersPerSecond) * redAllianceMultiplier);
    LinearVelocity yVelocity = Units.MetersPerSecond
        .of(-yAxis.getAsDouble() * constDrivetrain.REAL_DRIVE_SPEED.in(Units.MetersPerSecond) * redAllianceMultiplier);
    Pose2d closestPose;
    boolean isInAutoDriveZone = subDrivetrain.isInAutoDriveZone(
        Field.REEF_AUTO_DRIVE_MAX_DISTANCE,
        Field.getReefPositions(Field.isRedAlliance()).get());

    if (isInAutoDriveZone) {
      if (leftBranch == true) {
        closestPose = getLeftPos;
        subDriverStateMachine.setDriverState(DriverStateMachine.DriverState.REEF_AUTO_DRIVING_LEFT);
      } else {
        closestPose = getRightPos;
        subDriverStateMachine.setDriverState(DriverStateMachine.DriverState.REEF_AUTO_DRIVING_RIGHT);
      }
      subDrivetrain.autoAlign(isRedAlliance,
          closestPose,
          xVelocity,
          yVelocity,
          isOpenLoop,
          false,
          false);
    } else {
      closestPose = getAllPos;
      subDrivetrain.rotationalAlign(
          isRedAlliance,
          closestPose,
          xVelocity,
          yVelocity,
          isOpenLoop);
      subDriverStateMachine.setDriverState(DriverStateMachine.DriverState.REEF_ROTATION_SNAPPING);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subDrivetrain.neutralDriveOutputs();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
