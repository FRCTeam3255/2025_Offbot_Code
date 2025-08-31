// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.second_scoring_element;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismPositionGroup;
import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.Field;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Motion;
import frc.robot.subsystems.Rotors;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CleanLowWithCoral extends Command {
  /** Creates a new CleanLowWithCoral. */
  Motion globalMotion;
  Rotors globalRotors;
  StateMachine globalStateMachine;
  Pose2d closestPoseByRotation;
  Drivetrain globalDrivetrain;
  MechanismPositionGroup cleanLowWithCoral;

  public CleanLowWithCoral(StateMachine globalStateMachine, Motion subMotion, Rotors subRotors,
      Drivetrain subDrivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.globalStateMachine = globalStateMachine;
    this.globalMotion = subMotion;
    this.globalRotors = subRotors;
    globalDrivetrain = subDrivetrain;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (globalDrivetrain.isActionBackwards(
        Field.getAlgaePositions().get()) == true) {
      cleanLowWithCoral = constMechanismPositions.CLEAN_LOW_BACKWARDS;
    } else if (globalDrivetrain.isActionBackwards(
        Field.getAlgaePositions().get()) == false) {
      cleanLowWithCoral = constMechanismPositions.CLEAN_LOW_FORWARDS;
    }
    globalMotion.setAllPosition(cleanLowWithCoral);
    globalRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.CLEAN_ALGAE_SPEED);
    globalStateMachine.setRobotState(RobotState.CLEAN_LOW_WITH_CORAL);
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
