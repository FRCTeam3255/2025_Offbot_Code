// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.first_scoring_element;

import java.util.Set;

import frc.robot.Constants.MechanismPositionGroup;
import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.Field;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.commands.States.None;
import frc.robot.commands.States.hold_scoring_elements.HasAlgae;

public class CleanHigh extends StateCommand {
  MechanismPositionGroup cleanHigh;

  public CleanHigh() {
  }

  @Override
  protected Set<Class<? extends StateCommand>> getAllowedPreviousStates() {
    return Set.of(None.class, HasAlgae.class);
  }

  @Override

  public void initialize() {
    if (RobotContainer.subDrivetrain.isActionBackwards(Field.FieldElementGroups.ALGAE_POSES.getAll())) {
      cleanHigh = constMechanismPositions.CLEAN_HIGH_BACKWARDS;
    } else {
      cleanHigh = constMechanismPositions.CLEAN_HIGH_FORWARDS;
    }
    RobotContainer.subRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.CLEAN_ALGAE_SPEED);
    RobotContainer.subMotion.setAllPosition(cleanHigh);
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.subRotors.hasAlgae();
  }
}
