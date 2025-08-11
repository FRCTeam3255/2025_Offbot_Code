// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.frcteam3255.components.swerve.SN_SuperSwerve;
import com.frcteam3255.components.swerve.SN_SwerveModule;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.constDrivetrain;
import frc.robot.Constants.constField;
import frc.robot.Constants.constVision;
import frc.robot.RobotMap.mapDrivetrain;

@Logged
public class Drivetrain extends SN_SuperSwerve {
  StructPublisher<Pose2d> robotPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("/SmartDashboard/Drivetrain/Robot Pose", Pose2d.struct).publish();
  private static SN_SwerveModule[] modules = new SN_SwerveModule[] {
      new SN_SwerveModule(0, mapDrivetrain.FRONT_LEFT_DRIVE_CAN, mapDrivetrain.FRONT_LEFT_STEER_CAN,
          mapDrivetrain.FRONT_LEFT_ABSOLUTE_ENCODER_CAN, constDrivetrain.FRONT_LEFT_ABS_ENCODER_OFFSET,
          mapDrivetrain.CAN_BUS_NAME),
      new SN_SwerveModule(1, mapDrivetrain.FRONT_RIGHT_DRIVE_CAN, mapDrivetrain.FRONT_RIGHT_STEER_CAN,
          mapDrivetrain.FRONT_RIGHT_ABSOLUTE_ENCODER_CAN, constDrivetrain.FRONT_RIGHT_ABS_ENCODER_OFFSET,
          mapDrivetrain.CAN_BUS_NAME),
      new SN_SwerveModule(2, mapDrivetrain.BACK_LEFT_DRIVE_CAN, mapDrivetrain.BACK_LEFT_STEER_CAN,
          mapDrivetrain.BACK_LEFT_ABSOLUTE_ENCODER_CAN, constDrivetrain.BACK_LEFT_ABS_ENCODER_OFFSET,
          mapDrivetrain.CAN_BUS_NAME),
      new SN_SwerveModule(3, mapDrivetrain.BACK_RIGHT_DRIVE_CAN, mapDrivetrain.BACK_RIGHT_STEER_CAN,
          mapDrivetrain.BACK_RIGHT_ABSOLUTE_ENCODER_CAN, constDrivetrain.BACK_RIGHT_ABS_ENCODER_OFFSET,
          mapDrivetrain.CAN_BUS_NAME),
  };

  public Drivetrain() {
    super(
        constDrivetrain.SWERVE_CONSTANTS,
        modules,
        constDrivetrain.WHEELBASE,
        constDrivetrain.TRACK_WIDTH,
        mapDrivetrain.CAN_BUS_NAME,
        mapDrivetrain.PIGEON_CAN,
        constDrivetrain.MIN_STEER_PERCENT,
        constDrivetrain.DRIVE_CONFIG,
        constDrivetrain.STEER_CONFIG,
        constDrivetrain.CANCODER_CONFIG,
        VecBuilder.fill(
            constDrivetrain.MEASUREMENT_STD_DEVS_POS,
            constDrivetrain.MEASUREMENT_STD_DEVS_POS,
            constDrivetrain.MEASUREMENT_STD_DEV_HEADING),
        VecBuilder.fill(
            constVision.STD_DEVS_POS,
            constVision.STD_DEVS_POS,
            constVision.STD_DEVS_HEADING),
        constDrivetrain.AUTO.AUTO_DRIVE_PID,
        constDrivetrain.AUTO.AUTO_STEER_PID,
        constDrivetrain.TELEOP_AUTO_ALIGN.TELEOP_AUTO_ALIGN_CONTROLLER,
        constDrivetrain.TURN_SPEED,
        constDrivetrain.AUTO.ROBOT_CONFIG,
        () -> constField.isRedAlliance(),
        Robot.isSimulation());
  }

  @Override
  public void configure() {
    SN_SwerveModule.driveConfiguration = constDrivetrain.DRIVE_CONFIG;
    SN_SwerveModule.steerConfiguration = constDrivetrain.STEER_CONFIG;
    SN_SwerveModule.cancoderConfiguration = constDrivetrain.CANCODER_CONFIG;
    super.configure();
  }

  /**
   * Calculate which pose from an array has the closest rotation to the robot's
   * current pose. If multiple poses have the same rotation, the last one in the
   * list will be returned.
   * 
   * @param poses The list of poses to check
   * @return The last pose in the list with the closest rotation
   */
  public Pose2d getClosestPoseByRotation(List<Pose2d> poses) {
    Pose2d closestPose = poses.get(0);
    double smallestDifference = Math.abs(getRotation().minus(closestPose.getRotation()).getRadians());
    for (Pose2d pose : poses) {
      double difference = Math.abs(getRotation().minus(pose.getRotation()).getRadians());
      if (difference < smallestDifference) {
        smallestDifference = difference;
        closestPose = pose;
      }
    }
    return closestPose;
  }

  public Pose2d getRobotPose() {
    return getPose();
  }

  public Pose2d getFrontPose() {
    return getPose().plus(new Transform2d(Units.Inches.of(0), Units.Inches.of(14.5), new Rotation2d(0)));
  }

  public Pose2d getBackPose() {
    return getPose().plus(new Transform2d(Units.Inches.of(0), Units.Inches.of(-14.5), new Rotation2d(0)));
  }

  public Angle getRotationMeasure() {
    return Units.Degrees.of(getRotation().getDegrees());
  }

  public boolean reefActionBackwards(Pose2d closestPoseByRotation) {
    Distance backReefDistance;
    Distance frontReefDistance;
    closestPoseByRotation = getClosestPoseByRotation(constField.getReefPositions(true).get());
    backReefDistance = Units.Meters
        .of(getBackPose().getTranslation().getDistance(closestPoseByRotation.getTranslation()));
    frontReefDistance = Units.Meters
        .of(getFrontPose().getTranslation().getDistance(closestPoseByRotation.getTranslation()));
    if (backReefDistance.lt(frontReefDistance)) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    super.periodic();

    for (SN_SwerveModule mod : modules) {
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Desired Speed (FPS)",
          Units.Meters.convertFrom(Math.abs(getDesiredModuleStates()[mod.moduleNumber].speedMetersPerSecond),
              Units.Feet));
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Actual Speed (FPS)",
          Units.Meters.convertFrom(Math.abs(getActualModuleStates()[mod.moduleNumber].speedMetersPerSecond),
              Units.Feet));

      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Desired Angle (Degrees)",
          Math.abs(
              Units.Meters.convertFrom(getDesiredModuleStates()[mod.moduleNumber].angle.getDegrees(), Units.Feet)));
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Actual Angle (Degrees)",
          Math.abs(Units.Meters.convertFrom(getActualModuleStates()[mod.moduleNumber].angle.getDegrees(), Units.Feet)));

      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Offset Absolute Encoder Angle (Rotations)",
          mod.getAbsoluteEncoder());
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Absolute Encoder Raw Value (Rotations)",
          mod.getRawAbsoluteEncoder());
    }

    field.setRobotPose(getPose());
    robotPosePublisher.set(getPose());

    SmartDashboard.putData(field);
    SmartDashboard.putNumber("Drivetrain/Rotation", getRotationMeasure().in(Units.Degrees));
  }
}
