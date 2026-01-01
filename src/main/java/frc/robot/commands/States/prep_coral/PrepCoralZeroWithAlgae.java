// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.prep_coral;

import java.util.Set;

import frc.robot.Constants.constMechanismPositions;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.subsystems.RobotState;

public class PrepCoralZeroWithAlgae extends StateCommand {
  @Override
  protected Set<RobotState> getAllowedPreviousStates() {
    return Set.of(
        RobotState.HAS_ALGAE,
        RobotState.INTAKE_ALGAE_GROUND,
        RobotState.CLEAN_HIGH,
        RobotState.CLEAN_LOW);
  }

  @Override
  protected RobotState getDesiredState() {
    return RobotState.PREP_CORAL_ZERO_WITH_ALGAE;
  }

  @Override
  public void initialize() {
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.PREP_CORAL_ZERO);
  }
}
