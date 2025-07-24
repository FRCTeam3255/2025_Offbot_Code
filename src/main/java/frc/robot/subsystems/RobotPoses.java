// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constField;
import frc.robot.Constants.constIntake;

@Logged
public class RobotPoses extends SubsystemBase {

  Pose3d comp0Drivetrain = Pose3d.kZero;
  Pose3d comp1ElevatorStageOne = Pose3d.kZero;
  Pose3d comp2ElevatorCarriage = Pose3d.kZero;
  Pose3d comp3AlgaeIntake = Pose3d.kZero;
  Pose3d comp4Bumpers = Pose3d.kZero.plus(Constants.ROBOT_TO_BUMPERS);
  Pose3d coralPose = constField.POSES.SCORING_ELEMENT_NOT_COLLECTED;
  Pose3d algaePose = constField.POSES.SCORING_ELEMENT_NOT_COLLECTED;

  @NotLogged
  private Climber subClimber;
  @NotLogged
  private Drivetrain subDrivetrain;
  @NotLogged
  private Elevator subElevator;
  @NotLogged
  private Intake subIntake;

  /** Creates a new RobotPoses. */

  public RobotPoses(Climber subClimber,
      Drivetrain subDrivetrain, Elevator subElevator, Intake subIntake) {
    this.subClimber = subClimber;
    this.subDrivetrain = subDrivetrain;
    this.subElevator = subElevator;
    this.subIntake = subIntake;
  }

  @Override
  public void periodic() {
    Distance elevatorPos;
    Angle elevatorAngle;
    Angle intakAngle;

    // -- ROBOT POSITIONS --
    comp0Drivetrain = new Pose3d(subDrivetrain.getPose());

    // If we're in simulation, we can't log real mechanism data because they don't
    // exist. Instead, we'll log where we *want* the mechanisms to be and assume
    // they get there instantly.
    if (Robot.isSimulation()) {
      elevatorPos = subElevator.getLastDesiredElevatorPosition().div(2);
      elevatorAngle = subElevator.getLastDesiredElevatorPivotAngle();
      intakAngle = subIntake.getLastDesiredPivotAngle();
    } else {
      // Use real positions
      elevatorPos = (subElevator.getElevatorPosition().div(2));
      elevatorAngle = subElevator.getElevatorPivotAngle();
      intakAngle = subIntake.getPivotAngle();
    }

    // -- SCORING ELEMENTS --
    if (subIntake.hasAlgae()) {
      algaePose = comp0Drivetrain.plus(new Transform3d(Pose3d.kZero, comp3AlgaeIntake))
          .transformBy(constIntake.ALGAE_INTAKE_TO_ALGAE);
    } else {
      algaePose = constField.POSES.SCORING_ELEMENT_NOT_COLLECTED;
    }

    if (subIntake.hasCoral()) {
      coralPose = comp0Drivetrain.plus(new Transform3d(Pose3d.kZero, comp2ElevatorCarriage))
          .transformBy(constElevator.CARRIAGE_TO_CORAL);
    } else {
      coralPose = constField.POSES.SCORING_ELEMENT_NOT_COLLECTED;
    }
  }
}
