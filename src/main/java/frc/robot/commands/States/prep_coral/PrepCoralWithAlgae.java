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
public class PrepCoralWithAlgae extends Command {
  /** Creates a new PrepCoralWithAlgae. */
  Drivetrain globalDrivetrain;
  Motion globalMotion;
  Rotors globalRotors;
  StateMachine globalStateMachine;
  Distance globalHeight;
  Pose2d closestPoseByRotation;
  MechanismPositionGroup prepL2;
  MechanismPositionGroup prepL3;
  MechanismPositionGroup prepL4;

  public PrepCoralWithAlgae(StateMachine globalStateMachine, Motion subMotion, Rotors subRotors,
      Drivetrain subDrivetrain, Distance height) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalMotion = subMotion;
    globalRotors = subRotors;
    this.globalStateMachine = globalStateMachine;
    this.globalHeight = height;
    globalDrivetrain = subDrivetrain;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalDrivetrain.isActionBackwards(closestPoseByRotation,
        constField.getReefPositions(constField.isRedAlliance()).get());
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
    }
    if (globalHeight.equals(constMechanismPositions.ELEVATOR_CORAL_L1_HEIGHT)) {
      globalStateMachine.setRobotState(RobotState.PREP_CORAL_L1_WITH_ALGAE);
      globalMotion.setAllPosition(constMechanismPositions.PREP_CORAL_L1);
    } else if (globalHeight.equals(constMechanismPositions.ELEVATOR_CORAL_L2_HEIGHT)) {
      globalMotion.setAllPosition(prepL2);
      globalStateMachine.setRobotState(RobotState.PREP_CORAL_L2_WITH_ALGAE);
    } else if (globalHeight.equals(constMechanismPositions.ELEVATOR_CORAL_L3_HEIGHT)) {
      globalMotion.setAllPosition(prepL3);
      globalStateMachine.setRobotState(RobotState.PREP_CORAL_L3_WITH_ALGAE);
    } else if (globalHeight.equals(constMechanismPositions.ELEVATOR_CORAL_L4_HEIGHT)) {
      globalMotion.setAllPosition(prepL4);
      globalStateMachine.setRobotState(RobotState.PREP_CORAL_L4_WITH_ALGAE);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    globalDrivetrain.getClosestPoseByRotation(constField.getReefPositions(constField.isRedAlliance()).get());
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
