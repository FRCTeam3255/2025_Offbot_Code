// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.hold_scoring_elements;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.subsystems.Motion;
import frc.robot.subsystems.Rotors;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HasCoralAndAlgae extends Command {
  StateMachine globalStateMachine;

  public HasCoralAndAlgae() {
    globalStateMachine = StateMachine.getInstance();
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Rotors.getInstance().setHasCoralOverride(true);
    Rotors.getInstance().setHasAlgaeOverride(true);
    globalStateMachine.setRobotState(RobotState.HAS_CORAL_AND_ALGAE);
    Rotors.getInstance().setCoralIntakeMotorSpeed(0);
    Rotors.getInstance().setAlgaeIntakeMotorSpeed(constRotorsSpeeds.ALGAE_HOLD_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Motion.getInstance().setAllPosition(constMechanismPositions.HAS_SCORING_ELEMENTS);
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
