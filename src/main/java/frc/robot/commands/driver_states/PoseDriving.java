package frc.robot.commands.driver_states;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PoseDriveGroup;
import frc.robot.subsystems.DriverStateMachine;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.SwerveVelocity;

public class PoseDriving extends Command {
  Drivetrain subDrivetrain;
  DriverStateMachine subDriverStateMachine;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  PoseDriveGroup poseGroup;

  public PoseDriving(Drivetrain subDrivetrain, DriverStateMachine subDriverStateMachine,
      DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier rotationAxis, PoseDriveGroup poseGroup) {
    this.subDrivetrain = subDrivetrain;
    this.subDriverStateMachine = subDriverStateMachine;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    this.poseGroup = poseGroup;
    addRequirements(this.subDrivetrain, this.subDriverStateMachine);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Pose2d closestPose = subDrivetrain.getPose().nearest(poseGroup.targetPoseGroup);

    SwerveVelocity velocities = subDrivetrain.calculateVelocitiesFromInput(xAxis, yAxis, rotationAxis);

    boolean isInAutoDriveZone = subDrivetrain.isInAutoDriveZone(
        poseGroup.minDistanceBeforeDrive,
        closestPose);

    boolean backwardsAllowed = poseGroup.backwardsAllowed;

    if (subDrivetrain.isActionBackwards(poseGroup.targetPoseGroup) && backwardsAllowed) {
      closestPose = closestPose.rotateAround(closestPose.getTranslation(), Rotation2d.k180deg);
      velocities = new SwerveVelocity(-velocities.x, -velocities.y, velocities.rotation);
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
    return false;
  }
}