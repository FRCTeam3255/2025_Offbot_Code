package frc.robot.commands.driver_states;

import java.lang.Thread.State;
import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PoseDriveGroup;
import frc.robot.Constants.constField;
import frc.robot.Field;
import frc.robot.Field.FieldElementGroups;
import frc.robot.subsystems.DriverStateMachine;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.StateMachine;

@Logged
public class PoseDriving extends Command {
  Drivetrain subDrivetrain;
  DriverStateMachine subDriverStateMachine;
  StateMachine subStateMachine;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  PoseDriveGroup poseGroup;
  Pose2d closestPose;
  private boolean isPoseAligned = false;

  public PoseDriving(Drivetrain subDrivetrain, DriverStateMachine subDriverStateMachine, StateMachine subStateMachine,
      DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier rotationAxis, PoseDriveGroup poseGroup) {
    this.subDrivetrain = subDrivetrain;
    this.subStateMachine = subStateMachine;
    this.subDriverStateMachine = subDriverStateMachine;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    this.poseGroup = poseGroup;
    addRequirements(this.subDriverStateMachine);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    closestPose = subDrivetrain.getPose().nearest(poseGroup.targetPoseGroup);

    ChassisSpeeds velocities = subDrivetrain.calculateVelocitiesFromInput(xAxis, yAxis, rotationAxis);

    boolean isInAutoDriveZone = subDrivetrain.isInAutoDriveZone(
        poseGroup.minDistanceBeforeDrive,
        closestPose);

    boolean backwardsAllowed = poseGroup.backwardsAllowed;

    boolean isInPrepL2States = subStateMachine.getRobotState() == StateMachine.RobotState.PREP_CORAL_L2
        || subStateMachine.getRobotState() == StateMachine.RobotState.PREP_CORAL_L2_WITH_ALGAE;

    if (subDrivetrain.isActionBackwards(poseGroup.targetPoseGroup)
        && backwardsAllowed
        && !isInPrepL2States) {
      closestPose = closestPose.rotateAround(closestPose.getTranslation(), Rotation2d.k180deg);
      velocities.vxMetersPerSecond = -velocities.vxMetersPerSecond;
      velocities.vyMetersPerSecond = -velocities.vyMetersPerSecond;
    } else if (subDrivetrain.isActionBackwards(poseGroup.targetPoseGroup)
        && backwardsAllowed
        && isInPrepL2States) {
      if (poseGroup.targetPoseGroup.equals(FieldElementGroups.LEFT_REEF_POSES.getAll())) {
        closestPose = subDrivetrain.getPose().nearest(FieldElementGroups.LEFT_REEF_L2_BACKWARDS_POSES.getAll());
      } else if (poseGroup.targetPoseGroup.equals(FieldElementGroups.RIGHT_REEF_POSES.getAll())) {
        closestPose = subDrivetrain.getPose().nearest(FieldElementGroups.RIGHT_REEF_L2_BACKWARDS_POSES.getAll());
      }
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
