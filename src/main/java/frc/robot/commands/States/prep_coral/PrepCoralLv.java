// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.prep_coral;

import frc.robot.subsystems.StateMachine.RobotState;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StateMachine;
import frc.robot.Constants.constDrivetrain;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constField;
import frc.robot.Constants.constMechanismPositions;
import edu.wpi.first.units.measure.Angle;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepCoralLv extends Command {
  /** Creates a new PrepCoralLv. */
  Drivetrain globalDrivetrain;
  Elevator globalElevator;
  Intake globalIntake;
  StateMachine globalStateMachine;
  Distance globalHeight;
  Angle drivetrainRotation;
  Pose2d closestPoseByRotation;
  Distance reefDistance;

  public PrepCoralLv(StateMachine globalStateMachine, Elevator subElevator, Intake subIntake, Distance height,
      Drivetrain subDrivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalElevator = subElevator;
    globalIntake = subIntake;
    this.globalStateMachine = globalStateMachine;
    this.globalHeight = height;
    globalDrivetrain = subDrivetrain;
    addRequirements(globalStateMachine);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    closestPoseByRotation = globalDrivetrain.getClosestPoseByRotation(constField.getReefPositions(true).get());
    drivetrainRotation = globalDrivetrain.getRotationMeasure();
    reefDistance = Units.Meters
        .of(globalDrivetrain.getRobotPose().getTranslation().getDistance(closestPoseByRotation.getTranslation()));
    // if (reefDistance.lte(constDrivetrain.MINNIMUM_REEF_TOGGLE_DIFFERENCE) &&
    // drivetrainRotation.) {
    if (globalHeight.equals(constElevator.ELEVATOR_CORAL_L1_HEIGHT)) {
      globalElevator.setLiftPosition(constMechanismPositions.PREP_CORAL_L1_FORWARDS.liftHeight);
      globalElevator.setElevatorPivotAngle(constMechanismPositions.PREP_CORAL_L1_FORWARDS.pivotAngle);
      globalIntake.setWristPivotAngle(constMechanismPositions.PREP_CORAL_L1_FORWARDS.wristAngle);
      globalStateMachine.setRobotState(RobotState.PREP_CORAL_L1);
    } else if (globalHeight.equals(constElevator.ELEVATOR_CORAL_L2_HEIGHT)) {
      globalElevator.setLiftPosition(constMechanismPositions.PREP_CORAL_L2_FORWARDS.liftHeight);
      globalElevator.setElevatorPivotAngle(constMechanismPositions.PREP_CORAL_L2_FORWARDS.pivotAngle);
      globalIntake.setWristPivotAngle(constMechanismPositions.PREP_CORAL_L2_FORWARDS.wristAngle);
      globalStateMachine.setRobotState(RobotState.PREP_CORAL_L2);
    } else if (globalHeight.equals(constElevator.ELEVATOR_CORAL_L3_HEIGHT)) {
      globalElevator.setLiftPosition(constMechanismPositions.PREP_CORAL_L3_FORWARDS.liftHeight);
      globalElevator.setElevatorPivotAngle(constMechanismPositions.PREP_CORAL_L3_FORWARDS.pivotAngle);
      globalIntake.setWristPivotAngle(constMechanismPositions.PREP_CORAL_L3_FORWARDS.wristAngle);
      globalStateMachine.setRobotState(RobotState.PREP_CORAL_L3);
    } else if (globalHeight.equals(constElevator.ELEVATOR_CORAL_L4_HEIGHT)) {
      globalElevator.setLiftPosition(constMechanismPositions.PREP_CORAL_L4_FORWARDS.liftHeight);
      globalElevator.setElevatorPivotAngle(constMechanismPositions.PREP_CORAL_L4_FORWARDS.pivotAngle);
      globalIntake.setWristPivotAngle(constMechanismPositions.PREP_CORAL_L4_FORWARDS.wristAngle);
      globalStateMachine.setRobotState(RobotState.PREP_CORAL_L4);
    }
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    closestPoseByRotation = globalDrivetrain.getClosestPoseByRotation(constField.getReefPositions(true).get());
    drivetrainRotation = globalDrivetrain.getRotationMeasure();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
