// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.climbing;

import frc.robot.subsystems.Rotors;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.Motion;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climbing extends Command {
  Motion globalMotion;
  StateMachine globalStateMachine;
  Rotors globalRotors;

  public Climbing(StateMachine globalStateMachine, Motion subMotion, Rotors subRotors) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalRotors = subRotors;
    this.globalMotion = subMotion;
    this.globalStateMachine = globalStateMachine;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalStateMachine.setRobotState(RobotState.CLIMBING);
    globalMotion.setAllPosition(constMechanismPositions.CLIMBED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    globalMotion.setAllPosition(constMechanismPositions.LATCHED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return globalMotion.arePositionsAtSetPoint(constMechanismPositions.CLIMBED);
  }
}
