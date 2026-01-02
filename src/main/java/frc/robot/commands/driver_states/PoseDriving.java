package frc.robot.commands.driver_states;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PoseDriveGroup;
import frc.robot.Field.FieldElementGroups;
import frc.robot.RobotContainer;
import frc.robot.commands.States.prep_coral.PrepCoralL2;
import frc.robot.commands.States.prep_coral.PrepCoralWithAlgaeL2;

@Logged
public class PoseDriving extends Command {
  PoseDriveGroup poseGroup;
  Pose2d closestPose;
  public boolean isPoseAligned = false;

  public PoseDriving(PoseDriveGroup poseGroup) {
    this.poseGroup = poseGroup;
    addRequirements(RobotContainer.subDrivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    closestPose = RobotContainer.subDrivetrain.getPose().nearest(poseGroup.targetPoseGroup);
    RobotContainer.subDrivetrain.lastDesiredPoseGroup = poseGroup;

    ChassisSpeeds velocities = RobotContainer.subDrivetrain.calculateVelocitiesFromInput(
        RobotContainer.conDriver.axis_LeftY,
        RobotContainer.conDriver.axis_LeftX,
        RobotContainer.conDriver.axis_RightX,
        RobotContainer.conDriver.btn_RightBumper);

    boolean isInAutoDriveZone = RobotContainer.subDrivetrain.isInAutoDriveZone(
        poseGroup.minDistanceBeforeDrive,
        closestPose);

    boolean backwardsAllowed = poseGroup.backwardsAllowed;

    boolean isInPrepL2States = RobotContainer.getRobotState() == PrepCoralL2.class
        || RobotContainer.getRobotState() == PrepCoralWithAlgaeL2.class;

    if (RobotContainer.subDrivetrain.isActionBackwards(poseGroup.targetPoseGroup)
        && backwardsAllowed
        && !isInPrepL2States) {
      closestPose = closestPose.rotateAround(closestPose.getTranslation(), Rotation2d.k180deg);
      velocities.vxMetersPerSecond = -velocities.vxMetersPerSecond;
      velocities.vyMetersPerSecond = -velocities.vyMetersPerSecond;
    } else if (RobotContainer.subDrivetrain.isActionBackwards(poseGroup.targetPoseGroup)
        && backwardsAllowed
        && isInPrepL2States) {
      if (poseGroup.targetPoseGroup.equals(FieldElementGroups.LEFT_REEF_POSES.getAll())) {
        closestPose = RobotContainer.subDrivetrain.getPose()
            .nearest(FieldElementGroups.LEFT_REEF_L2_BACKWARDS_POSES.getAll());
      } else if (poseGroup.targetPoseGroup.equals(FieldElementGroups.RIGHT_REEF_POSES.getAll())) {
        closestPose = RobotContainer.subDrivetrain.getPose()
            .nearest(FieldElementGroups.RIGHT_REEF_L2_BACKWARDS_POSES.getAll());
      }
      velocities.vxMetersPerSecond = -velocities.vxMetersPerSecond;
      velocities.vyMetersPerSecond = -velocities.vyMetersPerSecond;
    }

    if (isInAutoDriveZone) {
      RobotContainer.subDrivetrain.autoAlign(
          closestPose,
          velocities,
          true,
          poseGroup.lockX,
          poseGroup.lockY);
      RobotContainer.setDriverState(poseGroup.driveState);
    } else {
      RobotContainer.subDrivetrain.rotationalAlign(
          closestPose,
          velocities,
          true);
      RobotContainer.setDriverState(poseGroup.snapState);
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.subDrivetrain.neutralDriveOutputs();
  }

  @Override
  public boolean isFinished() {
    isPoseAligned = RobotContainer.subDrivetrain.isAtPosition(closestPose, poseGroup.distanceTolerance) &&
        RobotContainer.subDrivetrain.isAtRotation(closestPose.getRotation(), poseGroup.rotationTolerance);
    return isPoseAligned;
  }
}
