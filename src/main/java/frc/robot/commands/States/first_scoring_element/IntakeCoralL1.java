// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.first_scoring_element;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.subsystems.Motion;
import frc.robot.subsystems.Rotors;
import frc.robot.subsystems.StateMachine;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoralL1 extends Command {
  /** Creates a new IntakeCoralL1. */
  public IntakeCoralL1() {
    addRequirements(StateMachine.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    StateMachine.getInstance().setRobotState(StateMachine.RobotState.INTAKE_CORAL_L1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Motion.getInstance().setAllPosition(constMechanismPositions.INTAKE_CORAL_L1);
    if (!Rotors.getInstance().seeL1Coral()) {
      Rotors.getInstance().setAlgaeIntakeMotorSpeed(constRotorsSpeeds.INTAKE_L1_SPEED);
    } else if (Rotors.getInstance().seeL1Coral() && !Rotors.getInstance().hasL1Coral()) {
      Rotors.getInstance().indexL1Coral(constRotorsSpeeds.INDEX_L1_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Rotors.getInstance().hasL1Coral();
  }
}
