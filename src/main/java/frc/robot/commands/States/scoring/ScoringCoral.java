// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.scoring;

import java.util.Set;

import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.subsystems.RobotState;

public class ScoringCoral extends StateCommand {

  public ScoringCoral() {
  }

  @Override
  protected Set<RobotState> getAllowedPreviousStates() {
    return Set.of(
        RobotState.PREP_CORAL_L2,
        RobotState.PREP_CORAL_L3,
        RobotState.PREP_CORAL_L4,
        RobotState.PREP_CORAL_ZERO,
        RobotState.PREP_CORAL_L1);
  }

  @Override
  protected RobotState getDesiredState() {
    return RobotState.SCORING_CORAL;
  }

  @Override
  public void initialize() {
    double speed = constRotorsSpeeds.SCORE_CORAL_SPEED;
    if (RobotContainer.subMotion.arePositionsAtSetPoint(constMechanismPositions.PREP_CORAL_L2_BACKWARDS)
        || RobotContainer.subMotion.arePositionsAtSetPoint(constMechanismPositions.PREP_CORAL_L3_BACKWARDS)
        || RobotContainer.subMotion.arePositionsAtSetPoint(constMechanismPositions.PREP_CORAL_L4_BACKWARDS)) {
      speed = -speed;
    }
    RobotContainer.subRotors.setCoralIntakeMotorSpeed(speed);
  }

  @Override
  public void execute() {
    // No continuous execution needed
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.subRotors.setHasCoralOverride(false);
  }
}
