// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.hold_scoring_elements;

import java.util.Set;

import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.RobotContainer;
import frc.robot.commands.StatefulCommand;
import frc.robot.commands.States.second_scoring_element.IntakeCoralGroundWithAlgae;
import frc.robot.commands.States.second_scoring_element.IntakeCoralStationWithAlgae;

public class HasCoralAndAlgae extends StatefulCommand {

  public HasCoralAndAlgae() {
  }

  @Override
  protected Set<Class<? extends StatefulCommand>> getAllowedPreviousStates() {
    return Set.of(
        IntakeCoralStationWithAlgae.class,
        IntakeCoralGroundWithAlgae.class);
  }

  @Override

  public void initialize() {
    RobotContainer.subRotors.setHasCoralOverride(true);
    RobotContainer.subRotors.setHasAlgaeOverride(true);
    RobotContainer.subRotors.setCoralIntakeMotorSpeed(0);
    RobotContainer.subRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.ALGAE_HOLD_SPEED);
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.HAS_SCORING_ELEMENTS);
  }
}
