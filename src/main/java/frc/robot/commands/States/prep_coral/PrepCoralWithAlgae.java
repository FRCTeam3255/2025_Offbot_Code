// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.prep_coral;

import frc.robot.subsystems.StateMachine.RobotState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Motion;
import frc.robot.subsystems.Rotors;
import frc.robot.subsystems.StateMachine;
import frc.robot.Constants.constMechanismPositions;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepCoralWithAlgae extends Command {
  /** Creates a new PrepCoralWithAlgae. */
  Motion globalMotion;
  Rotors globalRotors;
  StateMachine globalStateMachine;
  Distance globalDistance;

  public PrepCoralWithAlgae(StateMachine globalStateMachine, Motion subMotion, Rotors subRotors, Distance height) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalMotion = subMotion;
    globalRotors = subRotors;
    this.globalStateMachine = globalStateMachine;
    this.globalDistance = height;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (globalStateMachine.getRobotState() == RobotState.PREP_CORAL_L1_WITH_ALGAE) {
      globalMotion.setAllPosition(constMechanismPositions.PREP_CORAL_L1_FORWARDS);
      globalStateMachine.setRobotState(RobotState.PREP_CORAL_L1_WITH_ALGAE);
    } else if (globalStateMachine.getRobotState() == RobotState.PREP_CORAL_L2_WITH_ALGAE) {
      globalMotion.setAllPosition(constMechanismPositions.PREP_CORAL_L2_FORWARDS);
      globalStateMachine.setRobotState(RobotState.PREP_CORAL_L2_WITH_ALGAE);
    } else if (globalStateMachine.getRobotState() == RobotState.PREP_CORAL_L3_WITH_ALGAE) {
      globalMotion.setAllPosition(constMechanismPositions.PREP_CORAL_L3_FORWARDS);
      globalStateMachine.setRobotState(RobotState.PREP_CORAL_L3_WITH_ALGAE);
    } else if (globalStateMachine.getRobotState() == RobotState.PREP_CORAL_L4_WITH_ALGAE) {
      globalMotion.setAllPosition(constMechanismPositions.PREP_CORAL_L4_FORWARDS);
      globalStateMachine.setRobotState(RobotState.PREP_CORAL_L4_WITH_ALGAE);
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
