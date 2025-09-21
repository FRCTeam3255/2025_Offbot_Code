// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.first_scoring_element;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoralL1 extends Command {
  /** Creates a new IntakeCoralL1. */
  StateMachine globalStateMachine;
  Rotors globalRotors;
  Motion globalMotion;

  public IntakeCoralL1(StateMachine subStateMachine, Motion subMotion, Rotors subRotors) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalMotion = subMotion;
    globalRotors = subRotors;
    globalStateMachine = subStateMachine;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalStateMachine.setRobotState(StateMachine.RobotState.INTAKE_CORAL_L1);
    globalRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.INTAKE_L1_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    globalMotion.setAllPosition(constMechanismPositions.INTAKE_CORAL_L1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    globalRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.L1_CORAL_HOLD_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return globalRotors.hasL1Coral();
  }
}
