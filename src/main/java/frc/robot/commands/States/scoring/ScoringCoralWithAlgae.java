// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.scoring;

import java.util.Set;

import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.commands.States.prep_coral.PrepCoralWithAlgaeL2;
import frc.robot.commands.States.prep_coral.PrepCoralWithAlgaeL3;
import frc.robot.commands.States.prep_coral.PrepCoralWithAlgaeL4;
import frc.robot.commands.States.prep_coral.PrepCoralZeroWithAlgae;

public class ScoringCoralWithAlgae extends StateCommand {

  public ScoringCoralWithAlgae() {
  }

  @Override
  protected Set<Class<? extends StateCommand>> getAllowedPreviousStates() {
    return Set.of(
        PrepCoralWithAlgaeL2.class,
        PrepCoralWithAlgaeL3.class,
        PrepCoralWithAlgaeL4.class,
        PrepCoralZeroWithAlgae.class);
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
