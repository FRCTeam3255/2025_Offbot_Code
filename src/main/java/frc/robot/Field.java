// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class Field {
  public static Optional<Alliance> ALLIANCE = Optional.empty();
  public static final Distance FIELD_LENGTH = Units.Feet.of(57).plus(Units.Inches.of(6 + 7 / 8));
  public static final Distance FIELD_WIDTH = Units.Feet.of(26).plus(Units.Inches.of(5));

  public static final Distance REEF_AUTO_DRIVE_MAX_DISTANCE = Units.Meters.of(3);
  public static final Distance CORAL_STATION_AUTO_DRIVE_MAX_DISTANCE = Units.Meters.of(5);
  public static final Distance ALGAE_AUTO_DRIVE_MAX_DISTANCE = Units.Meters.of(3);
  public static final Distance PROCESSOR_AUTO_DRIVE_MAX_DISTANCE = Units.Meters.of(5);
  public static final Distance NET_AUTO_DRIVE_MAX_DISTANCE = Units.Meters.of(5);

  /**
   * Boolean that controls when the path will be mirrored for the red
   * alliance. This will flip the path being followed to the red side of the
   * field.
   * The origin will remain on the Blue side.
   * 
   * @return If we are currently on Red alliance. Will return false if no alliance
   *         is found
   */
  public static boolean isRedAlliance() {
    var alliance = ALLIANCE;
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  };

  public static class POSES {
    public static final Pose2d RESET_POSE = new Pose2d(3.169, 4.015, new Rotation2d());
    public static final Pose3d SCORING_ELEMENT_NOT_COLLECTED = new Pose3d(0, 0, -1, Rotation3d.kZero);

    // Branch Poses
    private static final Pose2d REEF_A = new Pose2d(3.171, 4.189, Rotation2d.kZero);
    private static final Pose2d REEF_B = new Pose2d(3.171, 3.863, Rotation2d.kZero);
    private static final Pose2d REEF_C = new Pose2d(3.688, 2.968, Rotation2d.fromDegrees(60));
    private static final Pose2d REEF_D = new Pose2d(3.975, 2.803, Rotation2d.fromDegrees(60));
    private static final Pose2d REEF_E = new Pose2d(5.001, 2.804, Rotation2d.fromDegrees(120));
    private static final Pose2d REEF_F = new Pose2d(5.285, 2.964, Rotation2d.fromDegrees(120));
    private static final Pose2d REEF_G = new Pose2d(5.805, 3.863, Rotation2d.k180deg);
    private static final Pose2d REEF_H = new Pose2d(5.805, 4.189, Rotation2d.k180deg);
    private static final Pose2d REEF_I = new Pose2d(5.288, 5.083, Rotation2d.fromDegrees(-120));
    private static final Pose2d REEF_J = new Pose2d(5.002, 5.248, Rotation2d.fromDegrees(-120));
    private static final Pose2d REEF_K = new Pose2d(3.972, 5.247, Rotation2d.fromDegrees(-60));
    private static final Pose2d REEF_L = new Pose2d(3.693, 5.079, Rotation2d.fromDegrees(-60));

    // Net Poses
    private static final Pose2d CENTER_LINE = new Pose2d(Units.Meters.of(8.850), Units.Meters.of(6.174),
        Rotation2d.fromDegrees(0));
    private static final Pose2d NET_LINE = new Pose2d(Units.Meters.of(7.588), Units.Meters.of(0),
        Rotation2d.fromDegrees(0));

    // algae poses
    public static final Pose2d ALGAE_AB = REEF_A.interpolate(REEF_B, 0.5);
    public static final Pose2d ALGAE_CD = REEF_C.interpolate(REEF_D, 0.5);
    public static final Pose2d ALGAE_EF = REEF_E.interpolate(REEF_F, 0.5);
    public static final Pose2d ALGAE_GH = REEF_G.interpolate(REEF_H, 0.5);
    public static final Pose2d ALGAE_IJ = REEF_I.interpolate(REEF_J, 0.5);
    public static final Pose2d ALGAE_KL = REEF_K.interpolate(REEF_L, 0.5);

    // Coral Station
    private static final Pose2d LEFT_CORAL_STATION_FAR = new Pose2d(1.64, 7.33, Rotation2d.fromDegrees(-55));
    private static final Pose2d LEFT_CORAL_STATION_NEAR = new Pose2d(0.71, 6.68, Rotation2d.fromDegrees(-55));
    private static final Pose2d RIGHT_CORAL_STATION_FAR = new Pose2d(1.61, 0.70, Rotation2d.fromDegrees(55));
    private static final Pose2d RIGHT_CORAL_STATION_NEAR = new Pose2d(0.64, 1.37, Rotation2d.fromDegrees(55));

    // Processor
    private static final Pose2d BLUE_PROCESSOR = new Pose2d(5.6, 0.896, Rotation2d.fromDegrees(-90));

    private static final Pose2d RED_PROCESSOR = getRedAlliancePose(BLUE_PROCESSOR);

    // Cages
    public static final Pose2d CAGE_1 = new Pose2d(7.783, 7.248, Rotation2d.fromDegrees(180));
    public static final Pose2d CAGE_2 = new Pose2d(7.783, 6.151, Rotation2d.fromDegrees(180));
    public static final Pose2d CAGE_3 = new Pose2d(7.783, 5.068, Rotation2d.fromDegrees(180));

    // --- lists ---
    private static final List<Pose2d> BLUE_NET_POSES = List.of(NET_LINE);
    private static final List<Pose2d> RED_NET_POSES = List.of(getRedPosesFromList(BLUE_NET_POSES));

    // -- REEF --
    private static final List<Pose2d> BLUE_REEF_POSES = List.of(REEF_A, REEF_B, REEF_C, REEF_D, REEF_E,
        REEF_F, REEF_G, REEF_H, REEF_I, REEF_J, REEF_K, REEF_L);
    private static final List<Pose2d> RED_REEF_POSES = List.of(getRedPosesFromList(BLUE_REEF_POSES));
    private static final List<Pose2d> BLUE_REEF_BACKWARDS_SCORING_POSES = List
        .of(getBackwardsScoringPosesFromList(BLUE_REEF_POSES));
    private static final List<Pose2d> RED_REEF_BACKWARDS_SCORING_POSES = List.of(getRedPosesFromList(
        BLUE_REEF_BACKWARDS_SCORING_POSES));
    private static final List<Pose2d> BLUE_LEFT_REEF_POSES = List.of(REEF_A, REEF_C, REEF_F, REEF_H, REEF_J,
        REEF_K);
    private static final List<Pose2d> RED_LEFT_REEF_POSES = List.of(getRedPosesFromList(BLUE_LEFT_REEF_POSES));
    private static final List<Pose2d> BLUE_RIGHT_REEF_POSES = List.of(REEF_B, REEF_D, REEF_E, REEF_G, REEF_I,
        REEF_L);
    private static final List<Pose2d> RED_RIGHT_REEF_POSES = List.of(getRedPosesFromList(BLUE_RIGHT_REEF_POSES));

    // -- Algae --
    private static final List<Pose2d> BLUE_ALGAE_POSES = List.of(ALGAE_AB, ALGAE_CD, ALGAE_EF, ALGAE_GH, ALGAE_IJ,
        ALGAE_KL);
    private static final List<Pose2d> RED_ALGAE_POSES = List.of(getRedPosesFromList(BLUE_ALGAE_POSES));

    // -- Coral Station --
    private static final List<Pose2d> BLUE_CORAL_STATION_POSES = List.of(LEFT_CORAL_STATION_FAR,
        LEFT_CORAL_STATION_NEAR, RIGHT_CORAL_STATION_FAR, RIGHT_CORAL_STATION_NEAR);
    private static final List<Pose2d> RED_CORAL_STATION_POSES = List.of(getRedPosesFromList(BLUE_CORAL_STATION_POSES));
    // -- Cage --
    private static final List<Pose2d> BLUE_CAGE_POSES = List.of(CAGE_1, CAGE_2, CAGE_3);
    private static final List<Pose2d> RED_CAGE_POSES = List.of(getRedPosesFromList(BLUE_CAGE_POSES));
  }

  public static Pose2d getRedAlliancePose(Pose2d bluePose) {
    return new Pose2d(FIELD_LENGTH.in(Units.Meters) - (bluePose.getX()),
        FIELD_WIDTH.in(Units.Meters) - bluePose.getY(),
        bluePose.getRotation().plus(Rotation2d.k180deg));
  }

  public static Pose2d[] getRedPosesFromList(List<Pose2d> bluePoseList) {
    Pose2d[] returnedPoses = new Pose2d[bluePoseList.size()];
    for (int i = 0; i < bluePoseList.size(); i++) {
      returnedPoses[i] = getRedAlliancePose(bluePoseList.get(i));
    }
    return returnedPoses;
  }

  private static Pose2d getBackwardsScoringPoses(Pose2d bluePose) {
    return new Pose2d(bluePose.getX(), bluePose.getY(),
        bluePose.getRotation().plus(Rotation2d.k180deg));
  }

  public static Pose2d[] getBackwardsScoringPosesFromList(List<Pose2d> bluePoseList) {
    Pose2d[] returnedPoses = new Pose2d[bluePoseList.size()];
    for (int i = 0; i < bluePoseList.size(); i++) {
      returnedPoses[i] = getBackwardsScoringPoses(bluePoseList.get(i));
    }
    return returnedPoses;
  }

  // -- REEF --
  public static Supplier<List<Pose2d>> getReefPositions(boolean onRed) {
    if (onRed) {
      return () -> POSES.RED_REEF_POSES;

    }
    return () -> POSES.BLUE_REEF_POSES;
  }

  public static Supplier<List<Pose2d>> getReefBackwardsScoringPositions(boolean onRed) {
    if (onRed) {
      return () -> POSES.RED_REEF_BACKWARDS_SCORING_POSES;
    }
    return () -> POSES.BLUE_REEF_BACKWARDS_SCORING_POSES;
  }

  public static Supplier<List<Pose2d>> getLeftReefPositions(boolean onRed) {
    if (onRed) {
      return () -> POSES.RED_LEFT_REEF_POSES;
    }
    return () -> POSES.BLUE_LEFT_REEF_POSES;
  }

  public static Supplier<List<Pose2d>> getRightReefPositions(boolean onRed) {
    if (onRed) {
      return () -> POSES.RED_RIGHT_REEF_POSES;
    }
    return () -> POSES.BLUE_RIGHT_REEF_POSES;
  }

  // -- ALGAE --
  public static Supplier<List<Pose2d>> getAlgaePositions(Boolean onRed) {
    if (onRed) {
      return () -> POSES.RED_ALGAE_POSES;
    }
    return () -> POSES.BLUE_ALGAE_POSES;
  }

  // -- NET --
  public static Supplier<List<Pose2d>> getNetPositions(boolean onRed) {
    if (onRed) {
      return () -> POSES.RED_NET_POSES;
    }
    return () -> POSES.BLUE_NET_POSES;
  }

  // -- CORAL STATION --
  public static Supplier<List<Pose2d>> getCoralStationPositions(boolean onRed) {
    if (onRed) {
      return () -> POSES.RED_CORAL_STATION_POSES;
    }
    return () -> POSES.BLUE_CORAL_STATION_POSES;
  }

  // -- CAGES --
  public static Supplier<List<Pose2d>> getCagePositions(boolean onRed) {
    if (onRed) {
      return () -> POSES.RED_CAGE_POSES;
    }
    return () -> POSES.BLUE_CAGE_POSES;
  }

  // -- PROCESSOR --
  public static Pose2d getProcessorPose(boolean onRed) {
    if (onRed) {
      return POSES.RED_PROCESSOR;
    }
    return POSES.BLUE_PROCESSOR;
  }

  public static final Pose2d WORKSHOP_STARTING_POSE = new Pose2d(5.98, 2.60, new Rotation2d(0));

}
