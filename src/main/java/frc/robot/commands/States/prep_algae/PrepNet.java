// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.prep_algae;

import java.util.Set;

import frc.robot.Constants.MechanismPositionGroup;
import frc.robot.Constants.constMechanismPositions;
import frc.robot.Field;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.commands.States.hold_scoring_elements.HasAlgae;
import frc.robot.commands.States.prep_algae.PrepAlgaeZero;
import frc.robot.commands.States.prep_algae.PrepProcessor;

public class PrepNet extends StateCommand {
  MechanismPositionGroup prepNet;

  public PrepNet() {
  }

  @Override
  protected Set<Class<? extends StateCommand>> getAllowedPreviousStates() {
    return Set.of(HasAlgae.class, PrepProcessor.class, PrepAlgaeZero.class);
  }

  @Override

  public void initialize() {
    if (RobotContainer.subDrivetrain.isActionBackwards(Field.FieldElementGroups.NET_POSES.getAll())) {
      prepNet = constMechanismPositions.PREP_ALGAE_NET_BACKWARDS;
    } else {
      prepNet = constMechanismPositions.PREP_ALGAE_NET_FORWARDS;
    }
    RobotContainer.subMotion.setAllPosition(prepNet);
  }
}
