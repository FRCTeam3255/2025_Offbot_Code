// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.hold_scoring_elements;

import java.util.Set;

import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.subsystems.RobotState;

public class HasAlgae extends StateCommand {

  public HasAlgae() {
  }

  @Override
  protected Set<RobotState> getAllowedPreviousStates() {
    return Set.of(
        RobotState.INTAKE_ALGAE_GROUND,
        RobotState.INTAKE_ALGAE_GROUND_WITH_CORAL,
        RobotState.CLEAN_HIGH,
        RobotState.CLEAN_LOW,
        RobotState.CLEAN_HIGH_WITH_CORAL,
        RobotState.CLEAN_LOW_WITH_CORAL);
  }

  @Override
  protected RobotState getDesiredState() {
    return RobotState.HAS_ALGAE;
  }

  @Override
  public void initialize() {
    if (RobotContainer.getRobotState() == RobotState.INTAKE_ALGAE_GROUND ||
        RobotContainer.getRobotState() == RobotState.INTAKE_ALGAE_GROUND_WITH_CORAL) {
      RobotContainer.subRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.ALGAE_HOLD_SPEED);
    } else if (RobotContainer.getRobotState() == RobotState.CLEAN_HIGH ||
        RobotContainer.getRobotState() == RobotState.CLEAN_LOW ||
        RobotContainer.getRobotState() == RobotState.CLEAN_HIGH_WITH_CORAL ||
        RobotContainer.getRobotState() == RobotState.CLEAN_LOW_WITH_CORAL) {
      RobotContainer.subRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.ALGAE_HARD_HOLD_SPEED);
    }
    RobotContainer.subRotors.setHasAlgaeOverride(true);
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.HAS_SCORING_ELEMENTS);
  }
}
