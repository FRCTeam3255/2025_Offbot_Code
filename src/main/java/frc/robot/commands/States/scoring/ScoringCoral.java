// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Motion;
import frc.robot.subsystems.Rotors;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoringCoral extends Command {
  StateMachine globalStateMachine = StateMachine.getInstance();
  Rotors globalRotors = Rotors.getInstance();
  Drivetrain globalDrivetrain = Drivetrain.getInstance();
  Motion globalMotion = Motion.getInstance();

  public ScoringCoral() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(globalStateMachine);
    addRequirements(globalRotors);
    addRequirements(globalDrivetrain);
    addRequirements(globalMotion);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double speed = constRotorsSpeeds.SCORE_CORAL_SPEED;
    if (globalMotion.arePositionsAtSetPoint(constMechanismPositions.PREP_CORAL_L2_BACKWARDS)
        || globalMotion.arePositionsAtSetPoint(constMechanismPositions.PREP_CORAL_L3_BACKWARDS)
        || globalMotion.arePositionsAtSetPoint(constMechanismPositions.PREP_CORAL_L4_BACKWARDS)) {
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
