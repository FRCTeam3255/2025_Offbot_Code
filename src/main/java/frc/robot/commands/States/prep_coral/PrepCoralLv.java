// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.prep_coral;

import frc.robot.subsystems.StateMachine.RobotState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StateMachine;
import frc.robot.Constants.constElevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepCoralLv extends Command {
  /** Creates a new PrepCoralLv. */
  StateMachine globalStateMachine;
  Distance globalDistance;

  public PrepCoralLv(StateMachine globalStateMachine, Elevator subElevator, Intake subIntake, Distance height) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.globalStateMachine = globalStateMachine;
    this.globalDistance = height;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (globalDistance.equals(constElevator.CORAL_L1_HEIGHT))
      globalStateMachine.setRobotState(RobotState.PREP_CORAL_L1);
    else if (globalDistance.equals(constElevator.CORAL_L2_HEIGHT))
      globalStateMachine.setRobotState(RobotState.PREP_CORAL_L2);
    else if (globalDistance.equals(constElevator.CORAL_L3_HEIGHT))
      globalStateMachine.setRobotState(RobotState.PREP_CORAL_L3);
    else if (globalDistance.equals(constElevator.CORAL_L4_HEIGHT))
      globalStateMachine.setRobotState(RobotState.PREP_CORAL_L4);
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
