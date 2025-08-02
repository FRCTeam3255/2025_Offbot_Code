
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.climbing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constClimber;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepClimb extends Command {
  /** Creates a new PrepClimb. */
  Elevator globalElevator;
  Intake globalIntake;
  Climber globalClimber;
  StateMachine globalStateMachine;

  public PrepClimb(StateMachine globalStateMachine, Climber subClimber, Intake subIntake, Elevator subElevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.globalStateMachine = globalStateMachine;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalElevator.setLiftPosition(null);
    globalElevator.setElevatorPivotAngle(null);
    globalIntake.setWristPivotAngle(null);
    globalClimber.setClimberMotorPercentOutput(constClimber.CLIMBER_MOTOR_PERCENT_OUTPUT);
    globalStateMachine.setRobotState(RobotState.PREP_CLIMB);
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
