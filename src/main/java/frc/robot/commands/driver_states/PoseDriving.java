package frc.robot.commands.driver_states;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PoseDriveGroup;
import frc.robot.subsystems.DriverStateMachine;
import frc.robot.subsystems.Drivetrain;

@Logged
public class PoseDriving extends Command {
  Drivetrain subDrivetrain;
  DriverStateMachine subDriverStateMachine;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  BooleanSupplier slowMode;
  PoseDriveGroup poseGroup;
  Pose2d closestPose;
  private boolean isPoseAligned = false;

  public PoseDriving(Drivetrain subDrivetrain, DriverStateMachine subDriverStateMachine,
  DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier rotationAxis, BooleanSupplier slowMode, PoseDriveGroup poseGroup) {
    this.subDrivetrain = subDrivetrain;
    this.subDriverStateMachine = subDriverStateMachine;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    this.poseGroup = poseGroup;
    this.slowMode = slowMode;
    addRequirements(this.subDriverStateMachine);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    closestPose = subDrivetrain.getPose().nearest(poseGroup.targetPoseGroup);

    ChassisSpeeds velocities = subDrivetrain.calculateVelocitiesFromInput(xAxis, yAxis, rotationAxis, slowMode);

    boolean isInAutoDriveZone = subDrivetrain.isInAutoDriveZone(
        poseGroup.minDistanceBeforeDrive,
        closestPose);

    boolean backwardsAllowed = poseGroup.backwardsAllowed;

    if (subDrivetrain.isActionBackwards(poseGroup.targetPoseGroup) && backwardsAllowed) {
      closestPose = closestPose.rotateAround(closestPose.getTranslation(), Rotation2d.k180deg);
      velocities.vxMetersPerSecond = -velocities.vxMetersPerSecond;
      velocities.vyMetersPerSecond = -velocities.vyMetersPerSecond;
    }

    if (isInAutoDriveZone) {
      subDrivetrain.autoAlign(
          closestPose,
          velocities,
          true,
          poseGroup.lockX,
          poseGroup.lockY);
      subDriverStateMachine.setDriverState(poseGroup.driveState);
    } else {
      subDrivetrain.rotationalAlign(
          closestPose,
          velocities,
          true);
      subDriverStateMachine.setDriverState(poseGroup.snapState);
    }
  }

  @Override
  public void end(boolean interrupted) {
    subDrivetrain.neutralDriveOutputs();
  }

  @Override
  public boolean isFinished() {
    isPoseAligned = subDrivetrain.isAtPosition(closestPose, poseGroup.distanceTolerance) &&
        subDrivetrain.isAtRotation(closestPose.getRotation(), poseGroup.rotationTolerance);
    return isPoseAligned;
  }
}