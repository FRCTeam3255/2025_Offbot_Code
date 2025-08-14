// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constField;

@Logged
public class RobotPoses extends SubsystemBase {
  /** Creates a new RobotPoses. */

  @NotLogged
  private Drivetrain subDrivetrain;
  @NotLogged
  private Motion subMotion;
  @NotLogged
  private Rotors subRotors;

  Pose3d modelDrivetrain = Pose3d.kZero;
  Pose3d model0Pivot = Pose3d.kZero;
  Pose3d model1ElevatorStage2 = Pose3d.kZero;
  Pose3d model2ElevatorCarriage = Pose3d.kZero;
  Pose3d model3Intake = Pose3d.kZero;
  Pose3d model4Climber = Pose3d.kZero;
  Pose3d coralPose = constField.POSES.SCORING_ELEMENT_NOT_COLLECTED;
  Pose3d algaePose = constField.POSES.SCORING_ELEMENT_NOT_COLLECTED;

  Transform3d elevatorTransform3d;
  Rotation3d pivotRotation3d;
  Rotation3d wristRotation3d;

  // Pivot Point Locations
  Transform3d wristPivotPoint = new Transform3d(
      Units.Inches.zero(),
      Units.Inches.of(16),
      Units.Inches.of(8),
      Rotation3d.kZero);
  Transform3d elevatorPivotPoint = new Transform3d(
      Units.Inches.zero(),
      Units.Inches.of(-9.8),
      Units.Inches.of(8),
      Rotation3d.kZero);

  public RobotPoses(Drivetrain subDrivetrain, Motion subMotion, Rotors subRotors) {
    this.subDrivetrain = subDrivetrain;
    this.subMotion = subMotion;
    this.subRotors = subRotors;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorTransform3d = new Transform3d(
        Units.Inches.zero(),
        subMotion.getLiftPosition().div(2),
        Units.Inches.zero(),
        Rotation3d.kZero);
    pivotRotation3d = new Rotation3d(
        subMotion.getPivotAngle(),
        Units.Degrees.zero(),
        Units.Degrees.zero());
    wristRotation3d = new Rotation3d(
        subMotion.getWristAngle(),
        Units.Degrees.zero(),
        Units.Degrees.zero());

    // Robot Positions
    modelDrivetrain = new Pose3d(subDrivetrain.getPose());

    model0Pivot = Pose3d.kZero.rotateAround(
        Pose3d.kZero.plus(elevatorPivotPoint).getTranslation(), pivotRotation3d);

    model1ElevatorStage2 = model0Pivot.transformBy(elevatorTransform3d);

    model2ElevatorCarriage = model1ElevatorStage2.transformBy(elevatorTransform3d);

    // Rotate intake around the end of the elevator carriage (adjust Z offset as
    // needed)
    model3Intake = model2ElevatorCarriage.rotateAround(
        model2ElevatorCarriage.plus(wristPivotPoint).getTranslation(), wristRotation3d);

    model4Climber = model0Pivot;
  }
}
