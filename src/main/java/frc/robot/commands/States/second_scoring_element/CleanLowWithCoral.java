// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.second_scoring_element;

import java.util.Set;

import frc.robot.Constants.MechanismPositionGroup;
import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.Field;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.subsystems.RobotState;

public class CleanLowWithCoral extends StateCommand {
  MechanismPositionGroup cleanLowWithCoral;

  public CleanLowWithCoral() {
  }

  @Override
  protected Set<RobotState> getAllowedPreviousStates() {
    return Set.of(
        RobotState.HAS_CORAL_AND_ALGAE,
        RobotState.SCORING_ALGAE_WITH_CORAL);
  }

  @Override
  protected RobotState getDesiredState() {
    return RobotState.CLEAN_LOW_WITH_CORAL;
  }

  @Override
  public void initialize() {
    if (RobotContainer.subDrivetrain.isActionBackwards(Field.FieldElementGroups.ALGAE_POSES.getAll())) {
      cleanLowWithCoral = constMechanismPositions.CLEAN_LOW_BACKWARDS;
    } else {
      cleanLowWithCoral = constMechanismPositions.CLEAN_LOW_FORWARDS;
    }
    RobotContainer.subRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.CLEAN_ALGAE_SPEED);
  }

  @Override
  public void execute() {
    RobotContainer.subMotion.setAllPosition(cleanLowWithCoral);
  }
}
