// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.first_scoring_element;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismPositionGroup;
import frc.robot.Constants.constField;
import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Motion;
import frc.robot.subsystems.Rotors;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CleanHigh extends Command {
  StateMachine globalStateMachine;
  Motion globalMotion;
  Rotors globalRotors;
  Pose2d closestPoseByRotation;
  Drivetrain globalDrivetrain;
  MechanismPositionGroup cleanHigh;

  public CleanHigh(StateMachine globalStateMachine, Motion subMotion, Rotors subRotors, Drivetrain subDrivetrain) {
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
    closestPoseByRotation = globalDrivetrain
        .getClosestPoseByRotation(constField.getAlgaePositions(constField.isRedAlliance()).get());
    if (globalDrivetrain.isActionBackwards(closestPoseByRotation,
        constField.getAlgaePositions(constField.isRedAlliance()).get()) == true) {
      cleanHigh = constMechanismPositions.CLEAN_HIGH_BACKWARDS;
    } else if (globalDrivetrain.isActionBackwards(closestPoseByRotation,
        constField.getAlgaePositions(constField.isRedAlliance()).get()) == false) {
      cleanHigh = constMechanismPositions.CLEAN_HIGH_FORWARDS;
    }
    globalMotion.setAllPosition(cleanHigh);
    globalRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.CLEAN_ALGAE_SPEED);
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
