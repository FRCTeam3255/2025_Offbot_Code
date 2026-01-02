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
import frc.robot.commands.StatefulCommand;
import frc.robot.commands.States.hold_scoring_elements.HasCoralAndAlgae;
import frc.robot.commands.States.scoring.ScoringAlgaeWithCoral;

public class CleanLowWithCoral extends StatefulCommand {
  MechanismPositionGroup cleanLowWithCoral;

  public CleanLowWithCoral() {
  }

  @Override
  protected Set<Class<? extends StatefulCommand>> getAllowedPreviousStates() {
    return Set.of(
        HasCoralAndAlgae.class,
        ScoringAlgaeWithCoral.class);
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
