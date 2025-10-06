// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.subsystems.Rotors;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoringAlgaeWithCoral extends Command {
  StateMachine globalStateMachine = StateMachine.getInstance();
  Rotors globalRotors = Rotors.getInstance();

  public ScoringAlgaeWithCoral() {
    // Use addRequirements() here to declare subsystem dependencies.
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
    globalStateMachine.setRobotState(RobotState.SCORING_ALGAE_WITH_CORAL);
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
