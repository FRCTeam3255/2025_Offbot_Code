// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.hold_scoring_elements;

import java.util.Set;

import frc.robot.Constants.constMechanismPositions;
import frc.robot.RobotContainer;
import frc.robot.commands.StatefulCommand;
import frc.robot.commands.States.first_scoring_element.IntakeCoralGround;
import frc.robot.commands.States.first_scoring_element.IntakeCoralL1;
import frc.robot.commands.States.first_scoring_element.IntakeCoralStation;
import frc.robot.commands.States.second_scoring_element.IntakeCoralGroundWithAlgae;
import frc.robot.commands.States.second_scoring_element.IntakeCoralStationWithAlgae;

public class HasCoral extends StatefulCommand {

  public HasCoral() {
  }

  @Override
  protected Set<Class<? extends StatefulCommand>> getAllowedPreviousStates() {
    return Set.of(
        IntakeCoralStation.class,
        IntakeCoralGround.class,
        IntakeCoralL1.class,
        IntakeCoralStationWithAlgae.class,
        IntakeCoralGroundWithAlgae.class);
  }

  @Override

  public void initialize() {
    RobotContainer.subRotors.setHasCoralOverride(true);
    RobotContainer.subRotors.setCoralIntakeMotorSpeed(0);
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.HAS_SCORING_ELEMENTS);
  }
}
