// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.scoring;

import java.util.Set;

import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.RobotContainer;
import frc.robot.commands.StatefulCommand;
import frc.robot.commands.States.prep_coral.PrepCoralL1;
import frc.robot.commands.States.prep_coral.PrepCoralL2;
import frc.robot.commands.States.prep_coral.PrepCoralL3;
import frc.robot.commands.States.prep_coral.PrepCoralL4;
import frc.robot.commands.States.prep_coral.PrepCoralZero;

public class ScoringCoral extends StatefulCommand {

  public ScoringCoral() {
  }

  @Override
  protected Set<Class<? extends StatefulCommand>> getAllowedPreviousStates() {
    return Set.of(
        PrepCoralL2.class,
        PrepCoralL3.class,
        PrepCoralL4.class,
        PrepCoralZero.class,
        PrepCoralL1.class);
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
