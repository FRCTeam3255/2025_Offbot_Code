// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.scoring;

import java.util.Set;

import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.subsystems.RobotState;

public class ScoringCoralWithAlgae extends StateCommand {

  public ScoringCoralWithAlgae() {
  }

  @Override
  protected Set<RobotState> getAllowedPreviousStates() {
    return Set.of(
        RobotState.PREP_CORAL_L2_WITH_ALGAE,
        RobotState.PREP_CORAL_L3_WITH_ALGAE,
        RobotState.PREP_CORAL_L4_WITH_ALGAE,
        RobotState.PREP_CORAL_ZERO_WITH_ALGAE);
  }

  @Override
  protected RobotState getDesiredState() {
    return RobotState.SCORING_CORAL_WITH_ALGAE;
  }

  @Override
  public void initialize() {
    RobotContainer.subRotors.setCoralIntakeMotorSpeed(constRotorsSpeeds.SCORE_CORAL_SPEED);
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
