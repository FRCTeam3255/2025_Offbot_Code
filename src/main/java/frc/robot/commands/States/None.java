// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States;

import java.util.Set;

import frc.robot.Constants.constMechanismPositions;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.subsystems.RobotState;

public class None extends StateCommand {

  public None() {
  }

  @Override
  protected Set<RobotState> getAllowedPreviousStates() {
    return Set.of(RobotState.PREP_CLIMB, RobotState.SCORING_CORAL, RobotState.SCORING_ALGAE,
        RobotState.INTAKE_CORAL_GROUND, RobotState.INTAKE_ALGAE_GROUND,
        RobotState.INTAKE_CORAL_STATION, RobotState.CLEAN_HIGH, RobotState.CLEAN_LOW,
        RobotState.EJECTING, RobotState.INTAKE_CORAL_L1, RobotState.SCORING_CORAL_L1,
        RobotState.CLIMBING);
  }

  @Override
  protected RobotState getDesiredState() {
    return RobotState.NONE;
  }

  @Override
  public void initialize() {
    RobotContainer.subRotors.setAlgaeIntakeMotorSpeed(0);
    RobotContainer.subRotors.setCoralIntakeMotorSpeed(0);
    RobotContainer.subRotors.setClimberMotorPercentOutput(0);
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.NONE);
  }
}
