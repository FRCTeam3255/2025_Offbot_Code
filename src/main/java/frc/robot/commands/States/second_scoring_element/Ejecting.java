// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.second_scoring_element;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.subsystems.Rotors;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Ejecting extends Command {
  StateMachine globalStateMachine;

  public Ejecting() {
    this.globalStateMachine = StateMachine.getInstance();
    addRequirements(globalStateMachine);
    addRequirements(Rotors.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Rotors.getInstance().setAlgaeIntakeMotorSpeed(constRotorsSpeeds.EJECTING_GAME_PIECE_SPEED);
    Rotors.getInstance().setCoralIntakeMotorSpeed(constRotorsSpeeds.EJECTING_GAME_PIECE_SPEED);
    globalStateMachine.setRobotState(RobotState.EJECTING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Rotors.getInstance().setHasCoralOverride(false);
    Rotors.getInstance().setHasAlgaeOverride(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
