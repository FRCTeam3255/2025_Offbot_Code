// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.second_scoring_element;

import frc.robot.subsystems.StateMachine.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.subsystems.Rotors;
import frc.robot.subsystems.StateMachine;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Ejecting extends Command {
  Rotors globalRotors;
  StateMachine globalStateMachine;

  public Ejecting(StateMachine globalStateMachine, Rotors subRotors) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalRotors = subRotors;
    this.globalStateMachine = globalStateMachine;
    addRequirements(globalStateMachine);
    addRequirements(globalRotors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.EJECTING_GAME_PIECE_SPEED);
    globalRotors.setCoralIntakeMotorSpeed(constRotorsSpeeds.EJECTING_GAME_PIECE_SPEED);
    globalStateMachine.setRobotState(RobotState.EJECTING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    globalRotors.setHasCoralOverride(false);
    globalRotors.setHasAlgaeOverride(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
