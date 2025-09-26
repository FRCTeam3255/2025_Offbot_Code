// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.scoring;

import frc.robot.subsystems.StateMachine.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StateMachine;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.Field;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Rotors;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoringCoral extends Command {
  StateMachine globalStateMachine;
  Rotors globalRotors;
  Drivetrain globalDrivetrain;

  public ScoringCoral(StateMachine globalStateMachine, Rotors subRotors, Drivetrain subDrivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalRotors = subRotors;
    this.globalStateMachine = globalStateMachine;
    globalDrivetrain = subDrivetrain;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double speed = constRotorsSpeeds.SCORE_CORAL_SPEED;
    if (globalDrivetrain.isActionBackwards(Field.FieldElementGroups.REEF_POSES.getAll())) {
      speed = -speed;
    }
    globalRotors.setCoralIntakeMotorSpeed(speed);
    globalStateMachine.setRobotState(RobotState.SCORING_CORAL);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    globalRotors.setHasCoralOverride(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
