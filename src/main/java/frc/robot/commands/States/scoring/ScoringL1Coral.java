// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.scoring;

import java.util.Set;

import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.commands.States.prep_coral.PrepCoralL1;

public class ScoringL1Coral extends StateCommand {

  public ScoringL1Coral() {
  }

  @Override
  protected Set<Class<? extends StateCommand>> getAllowedPreviousStates() {
    return Set.of(PrepCoralL1.class);
  }

  @Override

  public void initialize() {
    RobotContainer.subRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.L1_CORAL_SCORE_SPEED);
  }

  @Override
  public void execute() {
    // No continuous execution needed
  }
}
