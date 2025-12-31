// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.prep_algae;

import java.util.Set;

import frc.robot.Constants.constMechanismPositions;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.subsystems.RobotState;

public class PrepProcessor extends StateCommand {

  public PrepProcessor() {
  }

  @Override
  protected Set<RobotState> getAllowedPreviousStates() {
    return Set.of(
        RobotState.NONE,
        RobotState.INTAKE_ALGAE_GROUND,
        RobotState.CLEAN_HIGH,
        RobotState.CLEAN_LOW,
        RobotState.HAS_ALGAE);
  }

  @Override
  protected RobotState getDesiredState() {
    return RobotState.PREP_ALGAE_PROCESSOR;
  }

  @Override
  public void initialize() {
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.PREP_PROCESSOR);
  }
}
