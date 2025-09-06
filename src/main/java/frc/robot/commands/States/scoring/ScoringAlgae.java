// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.scoring;

import frc.robot.subsystems.StateMachine.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StateMachine;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.subsystems.Rotors;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoringAlgae extends Command {
  StateMachine globalStateMachine;
  Rotors globalRotors;

  public ScoringAlgae(StateMachine globalStateMachine, Rotors subRotors) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalRotors = subRotors;
    this.globalStateMachine = globalStateMachine;
    addRequirements(globalStateMachine);
    addRequirements(globalRotors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (globalStateMachine.getRobotState() == RobotState.PREP_ALGAE_NET) {
      globalRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.SCORE_ALGAE_NET_SPEED);
    } else if (globalStateMachine.getRobotState() == RobotState.PREP_ALGAE_PROCESSOR) {
      globalRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.SCORE_ALGAE_PROCESSOR_SPEED);
    }
    globalStateMachine.setRobotState(RobotState.SCORING_ALGAE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    globalRotors.setHasAlgaeOverride(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
