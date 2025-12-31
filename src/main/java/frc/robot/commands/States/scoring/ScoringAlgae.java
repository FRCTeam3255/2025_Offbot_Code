// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.scoring;

import java.util.Set;

import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.subsystems.RobotState;

public class ScoringAlgae extends StateCommand {

  public ScoringAlgae() {
  }

  @Override
  protected Set<RobotState> getAllowedPreviousStates() {
    return Set.of(
        RobotState.PREP_ALGAE_NET,
        RobotState.PREP_ALGAE_PROCESSOR,
        RobotState.PREP_ALGAE_ZERO);
  }

  @Override
  protected RobotState getDesiredState() {
    return RobotState.SCORING_ALGAE;
  }

  @Override
  public void initialize() {
    if (RobotContainer.getRobotState() == RobotState.PREP_ALGAE_NET) {
      RobotContainer.subRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.SCORE_ALGAE_NET_SPEED);
    } else if (RobotContainer.getRobotState() == RobotState.PREP_ALGAE_PROCESSOR) {
      RobotContainer.subRotors.setAlgaeIntakeMotorSpeed(0);
    }
  }

  @Override
  public void execute() {
    // No continuous execution needed
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.subRotors.setHasAlgaeOverride(false);
  }
}
