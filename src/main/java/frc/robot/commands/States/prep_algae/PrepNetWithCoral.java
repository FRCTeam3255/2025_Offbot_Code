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
import frc.robot.subsystems.RobotState;

public class PrepNetWithCoral extends StateCommand {
  MechanismPositionGroup prepNet;

  public PrepNetWithCoral() {
  }

  @Override
  protected Set<RobotState> getAllowedPreviousStates() {
    return Set.of(
        RobotState.HAS_CORAL,
        RobotState.CLEAN_HIGH_WITH_CORAL,
        RobotState.CLEAN_LOW_WITH_CORAL);
  }

  @Override
  protected RobotState getDesiredState() {
    return RobotState.PREP_ALGAE_NET_WITH_CORAL;
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
