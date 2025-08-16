// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.prep_coral;

import frc.robot.subsystems.StateMachine.RobotState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Motion;
import frc.robot.subsystems.Rotors;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.StateMachine;
import frc.robot.Constants.MechanismPositionGroup;
import frc.robot.Constants.constField;
import frc.robot.Constants.constMechanismPositions;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepCoralLv extends Command {
  /** Creates a new PrepCoralLv. */
  Drivetrain globalDrivetrain;
  Motion globalMotion;
  Rotors globalRotors;
  StateMachine globalStateMachine;
  Pose2d closestPoseByRotation;
  MechanismPositionGroup prepL2;
  MechanismPositionGroup prepL3;
  MechanismPositionGroup prepL4;
  int targetLevel;

  public PrepCoralLv(StateMachine globalStateMachine, Motion subMotion, Rotors subRotors, Drivetrain subDrivetrain,
      int level) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalMotion = subMotion;
    globalRotors = subRotors;
    this.globalStateMachine = globalStateMachine;
    globalDrivetrain = subDrivetrain;
    addRequirements(globalStateMachine);
    targetLevel = level;
  }

  @Override
  public void initialize() {
    closestPoseByRotation = globalDrivetrain
        .getClosestPoseByRotation(constField.getReefPositions(constField.isRedAlliance()).get());
    if (globalDrivetrain.isActionBackwards(closestPoseByRotation,
        constField.getReefPositions(constField.isRedAlliance()).get()) == true) {
      prepL2 = constMechanismPositions.PREP_CORAL_L2_BACKWARDS;
      prepL3 = constMechanismPositions.PREP_CORAL_L3_BACKWARDS;
      prepL4 = constMechanismPositions.PREP_CORAL_L4_BACKWARDS;
    } else if (globalDrivetrain.isActionBackwards(closestPoseByRotation,
        constField.getReefPositions(constField.isRedAlliance()).get()) == false) {
      prepL2 = constMechanismPositions.PREP_CORAL_L2_FORWARDS;
      prepL3 = constMechanismPositions.PREP_CORAL_L3_FORWARDS;
      prepL4 = constMechanismPositions.PREP_CORAL_L4_FORWARDS;
    } else if (targetLevel == 0) {
      globalMotion.setAllPosition(constMechanismPositions.PREP_CORAL_ZERO);
      globalStateMachine.setRobotState(RobotState.PREP_CORAL_ZERO);
    }
    if (targetLevel == 1) {
      globalStateMachine.setRobotState(RobotState.PREP_CORAL_L1);
      globalMotion.setAllPosition(constMechanismPositions.PREP_CORAL_L1);
    } else if (targetLevel == 2) {
      globalMotion.setAllPosition(prepL2);
      globalStateMachine.setRobotState(RobotState.PREP_CORAL_L2);
    } else if (targetLevel == 3) {
      globalMotion.setAllPosition(prepL3);
      globalStateMachine.setRobotState(RobotState.PREP_CORAL_L3);
    } else if (targetLevel == 4) {
      globalMotion.setAllPosition(prepL4);
      globalStateMachine.setRobotState(RobotState.PREP_CORAL_L4);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
