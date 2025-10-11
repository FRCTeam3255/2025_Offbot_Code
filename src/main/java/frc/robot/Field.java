package frc.robot;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Field {
  private static final Pose2d RESET_POSE = new Pose2d(3.169, 4.015, new Rotation2d());
  private static final Pose3d SCORING_ELEMENT_NOT_COLLECTED = new Pose3d(0, 0, -1, Rotation3d.kZero);

  public static Optional<Alliance> ALLIANCE = Optional.empty();
  public static final Distance FIELD_LENGTH = Units.Feet.of(57).plus(Units.Inches.of(6 + 7 / 8));
  public static final Distance FIELD_WIDTH = Units.Feet.of(26).plus(Units.Inches.of(5));

  public static final Distance REEF_AUTO_DRIVE_MAX_DISTANCE = Units.Meters.of(2);
  public static final Distance CORAL_STATION_AUTO_DRIVE_MAX_DISTANCE = Units.Meters.of(3);
  public static final Distance ALGAE_AUTO_DRIVE_MAX_DISTANCE = Units.Meters.of(2);
  public static final Distance PROCESSOR_AUTO_DRIVE_MAX_DISTANCE = Units.Meters.of(3);
  public static final Distance NET_AUTO_DRIVE_MAX_DISTANCE = Units.Meters.of(3);

  private static class FieldElements {
    private static final Pose2d REEF_CENTER = new Pose2d(4.5, 4.0, Rotation2d.kZero);
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

    // L2 Backwards
    private static final Pose2d REEF_A_L2_BACKWARDS = new Pose2d(2.848, 4.189, Rotation2d.k180deg);
    private static final Pose2d REEF_B_L2_BACKWARDS = new Pose2d(2.848, 3.863, Rotation2d.k180deg);
    private static final Pose2d REEF_C_L2_BACKWARDS = new Pose2d(3.522, 2.697, Rotation2d.fromDegrees(-120));
    private static final Pose2d REEF_D_L2_BACKWARDS = new Pose2d(3.801, 2.543, Rotation2d.fromDegrees(-120));
    private static final Pose2d REEF_E_L2_BACKWARDS = new Pose2d(5.157, 2.543, Rotation2d.fromDegrees(-60));
    private static final Pose2d REEF_F_L2_BACKWARDS = new Pose2d(5.446, 2.697, Rotation2d.fromDegrees(-60));
    private static final Pose2d REEF_G_L2_BACKWARDS = new Pose2d(6.110, 3.863, Rotation2d.kZero);
    private static final Pose2d REEF_H_L2_BACKWARDS = new Pose2d(6.110, 4.189, Rotation2d.kZero);
    private static final Pose2d REEF_I_L2_BACKWARDS = new Pose2d(5.456, 5.343, Rotation2d.fromDegrees(60));
    private static final Pose2d REEF_J_L2_BACKWARDS = new Pose2d(5.157, 5.497, Rotation2d.fromDegrees(60));
    private static final Pose2d REEF_K_L2_BACKWARDS = new Pose2d(3.820, 5.516, Rotation2d.fromDegrees(120));
    private static final Pose2d REEF_L_L2_BACKWARDS = new Pose2d(3.541, 5.343, Rotation2d.fromDegrees(120));

    // Net Poses
    private static final Pose2d CENTER_LINE = new Pose2d(Units.Meters.of(8.850), Units.Meters.of(6.174),
        Rotation2d.fromDegrees(0));
    private static final Pose2d NET_LINE = new Pose2d(Units.Meters.of(7.588), Units.Meters.of(6),
        Rotation2d.fromDegrees(0));
    private static final Pose2d NET_LINE_BACKWARD = new Pose2d(Units.Meters.of(10), Units.Meters.of(6),
        Rotation2d.fromDegrees(180));

    // algae poses
    private static final Pose2d ALGAE_AB = REEF_A.interpolate(REEF_B, 0.5);
    private static final Pose2d ALGAE_CD = REEF_C.interpolate(REEF_D, 0.5);
    private static final Pose2d ALGAE_EF = REEF_E.interpolate(REEF_F, 0.5);
    private static final Pose2d ALGAE_GH = REEF_G.interpolate(REEF_H, 0.5);
    private static final Pose2d ALGAE_IJ = REEF_I.interpolate(REEF_J, 0.5);
    private static final Pose2d ALGAE_KL = REEF_K.interpolate(REEF_L, 0.5);

    // Coral Station
    private static final Pose2d LEFT_CORAL_STATION_FAR = new Pose2d(1.64, 7.33, Rotation2d.fromDegrees(125));
    private static final Pose2d LEFT_CORAL_STATION_NEAR = new Pose2d(0.71, 6.68, Rotation2d.fromDegrees(125));
    private static final Pose2d RIGHT_CORAL_STATION_FAR = new Pose2d(1.61, 0.70, Rotation2d.fromDegrees(-125));
    private static final Pose2d RIGHT_CORAL_STATION_NEAR = new Pose2d(0.64, 1.37, Rotation2d.fromDegrees(-125));

    // Processor
    private static final Pose2d PROCESSOR = new Pose2d(5.6, 0.896, Rotation2d.fromDegrees(-90));

    // Cages
    private static final Pose2d CAGE_1 = new Pose2d(7.783, 7.248, Rotation2d.fromDegrees(180));
    private static final Pose2d CAGE_2 = new Pose2d(7.783, 6.151, Rotation2d.fromDegrees(180));
    private static final Pose2d CAGE_3 = new Pose2d(7.783, 5.068, Rotation2d.fromDegrees(180));
  }

  public static class FieldElementGroups {
    // --- groups wrapped for alliance handling ---

    public static final Pose2dAllianceSet NET_POSES = Pose2dAllianceSet.of(
        FieldElements.NET_LINE,
        FieldElements.NET_LINE_BACKWARD);

    // -- REEF --
    public static final Pose2dAllianceSet LEFT_REEF_POSES = Pose2dAllianceSet.of(
        FieldElements.REEF_A,
        FieldElements.REEF_C,
        FieldElements.REEF_F,
        FieldElements.REEF_H,
        FieldElements.REEF_J,
        FieldElements.REEF_K);

    public static final Pose2dAllianceSet RIGHT_REEF_POSES = Pose2dAllianceSet.of(
        FieldElements.REEF_B,
        FieldElements.REEF_D,
        FieldElements.REEF_E,
        FieldElements.REEF_G,
        FieldElements.REEF_I,
        FieldElements.REEF_L);

    public static final Pose2dAllianceSet LEFT_REEF_L2_BACKWARDS_POSES = Pose2dAllianceSet.of(
        FieldElements.REEF_A_L2_BACKWARDS,
        FieldElements.REEF_C_L2_BACKWARDS,
        FieldElements.REEF_F_L2_BACKWARDS,
        FieldElements.REEF_H_L2_BACKWARDS,
        FieldElements.REEF_J_L2_BACKWARDS,
        FieldElements.REEF_K_L2_BACKWARDS);

    public static final Pose2dAllianceSet RIGHT_REEF_L2_BACKWARDS_POSES = Pose2dAllianceSet.of(
        FieldElements.REEF_B_L2_BACKWARDS,
        FieldElements.REEF_D_L2_BACKWARDS,
        FieldElements.REEF_E_L2_BACKWARDS,
        FieldElements.REEF_G_L2_BACKWARDS,
        FieldElements.REEF_I_L2_BACKWARDS,
        FieldElements.REEF_L_L2_BACKWARDS);

    public static final Pose2dAllianceSet REEF_POSES = Pose2dAllianceSet.concat(
        LEFT_REEF_POSES,
        RIGHT_REEF_POSES);

    // Coral Stations
    public static final Pose2dAllianceSet FAR_CORAL_STATION_POSES = Pose2dAllianceSet.of(
        FieldElements.LEFT_CORAL_STATION_FAR,
        FieldElements.RIGHT_CORAL_STATION_FAR);

    public static final Pose2dAllianceSet NEAR_CORAL_STATION_POSES = Pose2dAllianceSet.of(
        FieldElements.LEFT_CORAL_STATION_NEAR,
        FieldElements.RIGHT_CORAL_STATION_NEAR);

    public static final Pose2dAllianceSet CORAL_STATION_POSES = Pose2dAllianceSet.concat(
        FAR_CORAL_STATION_POSES,
        NEAR_CORAL_STATION_POSES);

    // Algae
    public static final Pose2dAllianceSet ALGAE_POSES = Pose2dAllianceSet.of(
        FieldElements.ALGAE_AB,
        FieldElements.ALGAE_CD,
        FieldElements.ALGAE_EF,
        FieldElements.ALGAE_GH,
        FieldElements.ALGAE_IJ,
        FieldElements.ALGAE_KL);

    // CAGES
    public static final Pose2dAllianceSet CAGE_POSES = Pose2dAllianceSet.of(
        FieldElements.CAGE_1,
        FieldElements.CAGE_2,
        FieldElements.CAGE_3);

    // PROCESSOR
    public static final Pose2dAllianceSet PROCESSOR_POSES = Pose2dAllianceSet.of(
        FieldElements.PROCESSOR);
  }

  // Wrapper for blue-side Pose2d arrays with helpers for red/all
  public static final class Pose2dAllianceSet {
    private final List<Pose2d> blue;
    private final List<Pose2d> red;
    private final List<Pose2d> all;

    Pose2dAllianceSet(Pose2d... bluePoses) {
      this.blue = List.of(bluePoses);
      this.red = this.blue.stream().map(Pose2dAllianceSet::toRed).toList();

      var combined = new java.util.ArrayList<Pose2d>(this.blue.size() * 2);
      combined.addAll(this.blue);
      combined.addAll(this.red);
      this.all = List.copyOf(combined);
    }

    public List<Pose2d> getBlue() {
      return blue;
    }

    public List<Pose2d> getRed() {
      return red;
    }

    public List<Pose2d> getAll() {
      return all;
    }

    private static Pose2d toRed(Pose2d bluePose) {
      return new Pose2d(
          FIELD_LENGTH.in(Units.Meters) - bluePose.getX(),
          FIELD_WIDTH.in(Units.Meters) - bluePose.getY(),
          bluePose.getRotation().plus(Rotation2d.k180deg));
    }

    static Pose2dAllianceSet of(Pose2d... bluePoses) {
      return new Pose2dAllianceSet(bluePoses);
    }

    static Pose2dAllianceSet concat(Pose2dAllianceSet... sets) {
      var combined = new java.util.ArrayList<Pose2d>();
      for (Pose2dAllianceSet set : sets) {
        combined.addAll(set.getBlue());
      }
      return new Pose2dAllianceSet(combined.toArray(new Pose2d[0]));
    }
  }
}
