// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States;

import frc.robot.subsystems.StateMachine.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constLED;
import frc.robot.Constants.constMechanismPositions;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class None extends Command {
  Motion globalMotion;
  Rotors globalRotors;
  LED globalLED;
  StateMachine globalStateMachine;

  public None(StateMachine globalStateMachine, Motion subMotion, Rotors subRotors, LED subLED) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalMotion = subMotion;
    globalRotors = subRotors;
    globalLED = subLED;
    this.globalStateMachine = globalStateMachine;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalRotors.setAlgaeIntakeMotorSpeed(0);
    globalRotors.setCoralIntakeMotorSpeed(0);
    globalRotors.setClimberMotorPercentOutput(0);
    globalStateMachine.setRobotState(RobotState.NONE);
    // globalLED.setLED(constLED.NONE_ANIMATION, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    globalMotion.setAllPosition(constMechanismPositions.NONE);
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
