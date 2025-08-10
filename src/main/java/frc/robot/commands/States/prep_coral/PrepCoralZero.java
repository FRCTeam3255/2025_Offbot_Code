// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.prep_coral;

import frc.robot.subsystems.StateMachine.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constMechanismPositions;
import frc.robot.subsystems.Motion;
import frc.robot.subsystems.Rotors;
import frc.robot.subsystems.StateMachine;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepCoralZero extends Command {
  Motion globalMotion;
  Rotors globalRotors;
  StateMachine globalStateMachine;

  public PrepCoralZero(StateMachine globalStateMachine, Motion subMotion, Rotors subRotors) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalMotion = subMotion;
    globalRotors = subRotors;
    this.globalStateMachine = globalStateMachine;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalMotion.setAllPosition(constMechanismPositions.PREP_CORAL_ZERO);
    globalStateMachine.setRobotState(RobotState.PREP_CORAL_ZERO);
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
