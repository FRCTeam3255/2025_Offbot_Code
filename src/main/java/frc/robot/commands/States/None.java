// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States;

import frc.robot.subsystems.StateMachine.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class None extends Command {
  Elevator globalElevator;
  Intake globalIntake;
  StateMachine globalStateMachine;

  public None(StateMachine globalStateMachine, Elevator subElevator, Intake subIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalElevator = subElevator;
    globalIntake = subIntake;
    this.globalStateMachine = globalStateMachine;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalElevator.setElevatorPivotAngle(null);
    globalElevator.setLiftPosition(null);
    globalIntake.setWristPivotAngle(null);
    globalIntake.setIntakeMotorNeutralMode(null);
    globalStateMachine.setRobotState(RobotState.NONE);
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
