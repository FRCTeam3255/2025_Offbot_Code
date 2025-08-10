// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.prep_algae;

import frc.robot.subsystems.StateMachine.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismPositionGroup;
import frc.robot.Constants.constMechanismPositions;
import frc.robot.subsystems.Motion;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Rotors;
import frc.robot.subsystems.StateMachine;
import frc.robot.Constants.constField;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepNetWithCoral extends Command {
  Motion globalMotion;
  Rotors globalRotors;
  StateMachine globalStateMachine;

  Drivetrain globalDrivetrain;
  Pose2d closestPoseByRotation;

  Distance backNetDistance;
  Distance frontNetDistance;

  public PrepNetWithCoral(StateMachine globalStateMachine, Motion subMotion, Rotors subRotors,
      Drivetrain subDrivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalMotion = subMotion;
    globalRotors = subRotors;
    globalDrivetrain = subDrivetrain;
    this.globalStateMachine = globalStateMachine;
    addRequirements(globalStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    closestPoseByRotation = globalDrivetrain.getClosestPoseByRotation(constField.getNetPositions(true).get());
    backNetDistance = Units.Meters
        .of(globalDrivetrain.getBackPose().getTranslation().getDistance(closestPoseByRotation.getTranslation()));
    frontNetDistance = Units.Meters
        .of(globalDrivetrain.getFrontPose().getTranslation().getDistance(closestPoseByRotation.getTranslation()));
    if (frontNetDistance.lt(backNetDistance)) {
      globalMotion.setAllPosition(constMechanismPositions.PREP_ALGAE_NET_FORWARDS);
      globalStateMachine.setRobotState(RobotState.PREP_ALGAE_NET_WITH_CORAL);
    } else if (backNetDistance.lt(frontNetDistance)) {
      globalMotion.setAllPosition(constMechanismPositions.PREP_ALGAE_NET_BACKWARDS);
      globalStateMachine.setRobotState(RobotState.PREP_ALGAE_NET_WITH_CORAL);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    closestPoseByRotation = globalDrivetrain.getClosestPoseByRotation(constField.getNetPositions(true).get());
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
