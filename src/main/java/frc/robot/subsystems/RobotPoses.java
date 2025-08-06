// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constField;

@Logged
public class RobotPoses extends SubsystemBase {
  /** Creates a new RobotPoses. */

  @NotLogged
  private Drivetrain subDrivetrain;
  @NotLogged
  private Intake subIntake;
  @NotLogged
  private Elevator subElevator;

  Pose3d modelDrivetrain = Pose3d.kZero;
  Pose3d model0Pivot = Pose3d.kZero;
  Pose3d model1ElevatorStage2 = Pose3d.kZero;
  Pose3d model2ElevatorCarriage = Pose3d.kZero;
  Pose3d model3Intake = Pose3d.kZero;
  Pose3d coralPose = constField.POSES.SCORING_ELEMENT_NOT_COLLECTED;
  Pose3d algaePose = constField.POSES.SCORING_ELEMENT_NOT_COLLECTED;

  // Pose3d comp1Bumpers = Pose3d.kZero.plus(Constants.ROBOT_TO_BUMPERS);

  public RobotPoses(Drivetrain subDrivetrain, Elevator subElevator, Intake subIntake) {
    this.subDrivetrain = subDrivetrain;
    this.subElevator = subElevator;
    this.subIntake = subIntake;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Distance elevatorPos;
    Angle pivotAngle;
    Angle wristAngle;

    elevatorPos = subElevator.getLastDesiredLiftPosition().div(2);
    pivotAngle = subElevator.getLastDesiredPivotAngle();
    wristAngle = subIntake.getLastDesiredWristPivotAngle();

    // Robot Positions
    modelDrivetrain = new Pose3d(subDrivetrain.getPose());

    model0Pivot = Pose3d.kZero.rotateAround(
        new Translation3d(
            Units.Inches.zero(),
            Units.Inches.of(-9.8),
            Units.Inches.of(8)),
        new Rotation3d(
            pivotAngle,
            Units.Degrees.zero(),
            Units.Degrees.zero()));

    model1ElevatorStage2 = model0Pivot.transformBy(
        new Transform3d(
            new Translation3d(
                Units.Inches.of(0),
                elevatorPos.plus(Units.Inches.of(0)),
                Units.Inches.of(0)),
            Rotation3d.kZero));

    model2ElevatorCarriage = model1ElevatorStage2.transformBy(
        new Transform3d(
            new Translation3d(
                Units.Inches.of(0),
                elevatorPos.plus(Units.Inches.of(0)),
                Units.Inches.of(0)),
            Rotation3d.kZero));

    model3Intake = model2ElevatorCarriage.rotateAround(
        model2ElevatorCarriage.getTranslation().plus(
            new Translation3d(
                Units.Inches.of(0),
                Units.Inches.of(0),
                Units.Inches.of(0))),
        new Rotation3d(
            wristAngle,
            Units.Degrees.zero(),
            Units.Degrees.zero()));
  }
}
