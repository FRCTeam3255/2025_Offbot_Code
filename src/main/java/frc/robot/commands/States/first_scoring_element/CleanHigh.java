// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.first_scoring_element;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CleanHigh extends Command {
  StateMachine globalStateMachine;
  Elevator globalElevator;
  Intake globalIntake;

  public CleanHigh(StateMachine globalStateMachine, Elevator subElevator, Intake subIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.globalStateMachine = globalStateMachine;
    this.globalElevator = subElevator;
    this.globalIntake = subIntake;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalElevator.setLiftPosition(constElevator.ELEVATOR_CLEAN_HIGH_HIEGHT);
    globalElevator.setElevatorPivotAngle(constElevator.ELEVATOR_CLEAN_HIGH_ANGLE);
    globalIntake.setWristPivotAngle(constIntake.INTAKE_CLEAN_HIGH_ANGLE);
    globalIntake.setAlgaeIntakeMotorSpeed(constIntake.INTAKE_ALGAE_SPEED);
    globalStateMachine.setRobotState(RobotState.CLEAN_HIGH);
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
