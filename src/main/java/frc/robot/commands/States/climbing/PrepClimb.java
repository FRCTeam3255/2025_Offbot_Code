// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.climbing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotors;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.subsystems.Motion;
import frc.robot.subsystems.Rotors;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepClimb extends Command {
  /** Creates a new PrepClimb. */
  Motion globalMotion;
  Rotors globalRotors;
  StateMachine globalStateMachine;

  public PrepClimb(StateMachine globalStateMachine, Motion subMotion, Rotors subRotors) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalMotion = subMotion;
    globalRotors = subRotors;
    this.globalStateMachine = globalStateMachine;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalRotors.setClimberMotorPercentOutput(constRotorsSpeeds.CLIMBER_MOTOR_PERCENT_OUTPUT); // Assuming this is still
                                                                                               // needed
    globalStateMachine.setRobotState(RobotState.PREP_CLIMB);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    globalMotion.setAllPosition(constMechanismPositions.PREP_CLIMB);
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
