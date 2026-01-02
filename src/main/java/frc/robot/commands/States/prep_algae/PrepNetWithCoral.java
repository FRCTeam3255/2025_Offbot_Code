// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.prep_algae;

import java.util.Set;

import frc.robot.Constants.MechanismPositionGroup;
import frc.robot.Constants.constMechanismPositions;
import frc.robot.Field;
import frc.robot.RobotContainer;
import frc.robot.commands.StatefulCommand;
import frc.robot.commands.States.hold_scoring_elements.HasCoral;
import frc.robot.commands.States.second_scoring_element.CleanHighWithCoral;
import frc.robot.commands.States.second_scoring_element.CleanLowWithCoral;

public class PrepNetWithCoral extends StatefulCommand {
  MechanismPositionGroup prepNet;

  public PrepNetWithCoral() {
  }

  @Override
  protected Set<Class<? extends StatefulCommand>> getAllowedPreviousStates() {
    return Set.of(
        HasCoral.class,
        CleanHighWithCoral.class,
        CleanLowWithCoral.class);
  }

  @Override

  public void initialize() {
    if (RobotContainer.subDrivetrain.isActionBackwards(
        Field.FieldElementGroups.NET_POSES.getAll())) {
      prepNet = constMechanismPositions.PREP_ALGAE_NET_BACKWARDS;
    } else {
      prepNet = constMechanismPositions.PREP_ALGAE_NET_FORWARDS;
    }
    RobotContainer.subMotion.setAllPosition(prepNet);
  }
}
