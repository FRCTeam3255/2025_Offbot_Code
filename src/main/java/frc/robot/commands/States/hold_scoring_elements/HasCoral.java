// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.hold_scoring_elements;

import java.util.Set;

import frc.robot.Constants.constMechanismPositions;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.subsystems.RobotState;

public class HasCoral extends StateCommand {

  public HasCoral() {
  }

  @Override
  protected Set<RobotState> getAllowedPreviousStates() {
    return Set.of(
        RobotState.INTAKE_CORAL_STATION,
        RobotState.INTAKE_CORAL_GROUND,
        RobotState.INTAKE_CORAL_L1,
        RobotState.INTAKE_CORAL_STATION_WITH_ALGAE,
        RobotState.INTAKE_CORAL_GROUND_WITH_ALGAE);
  }

  @Override
  protected RobotState getDesiredState() {
    return RobotState.HAS_CORAL;
  }

  @Override
  public void initialize() {
    RobotContainer.subRotors.setHasCoralOverride(true);
    RobotContainer.subRotors.setCoralIntakeMotorSpeed(0);
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.HAS_SCORING_ELEMENTS);
  }
}
