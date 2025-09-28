// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.hold_scoring_elements;

import frc.robot.subsystems.StateMachine.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StateMachine;
import frc.robot.Constants.*;
import frc.robot.subsystems.Rotors;
import frc.robot.subsystems.Motion;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HasAlgae extends Command {
  StateMachine globalStateMachine;
  Motion globalMotion;
  Rotors globalRotors;

  public HasAlgae(StateMachine globalStateMachine, Motion subMotion, Rotors subRotors) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.globalStateMachine = globalStateMachine;
    globalMotion = subMotion;
    globalRotors = subRotors;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (globalStateMachine.getRobotState() == RobotState.INTAKE_ALGAE_GROUND ||
        globalStateMachine.getRobotState() == RobotState.INTAKE_ALGAE_GROUND_WITH_CORAL) {
      globalRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.ALGAE_HOLD_SPEED);
    } else if (globalStateMachine.getRobotState() == RobotState.CLEAN_HIGH ||
        globalStateMachine.getRobotState() == RobotState.CLEAN_LOW ||
        globalStateMachine.getRobotState() == RobotState.CLEAN_HIGH_WITH_CORAL ||
        globalStateMachine.getRobotState() == RobotState.CLEAN_LOW_WITH_CORAL) {
      globalRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.ALGAE_HARD_HOLD_SPEED);
    }
    globalStateMachine.setRobotState(RobotState.HAS_ALGAE);
    globalRotors.setHasAlgaeOverride(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    globalMotion.setAllPosition(constMechanismPositions.NONE);
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
