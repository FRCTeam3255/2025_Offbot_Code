// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.prep_algae;

import java.util.Set;

import frc.robot.Constants.constMechanismPositions;
import frc.robot.RobotContainer;
import frc.robot.commands.StatefulCommand;
import frc.robot.commands.States.None;
import frc.robot.commands.States.first_scoring_element.CleanHigh;
import frc.robot.commands.States.first_scoring_element.CleanLow;
import frc.robot.commands.States.first_scoring_element.IntakeAlgaeGround;
import frc.robot.commands.States.hold_scoring_elements.HasAlgae;

public class PrepAlgaeZero extends StatefulCommand {

  public PrepAlgaeZero() {
  }

  @Override
  protected Set<Class<? extends StatefulCommand>> getAllowedPreviousStates() {
    return Set.of(
        None.class,
        IntakeAlgaeGround.class,
        CleanHigh.class,
        CleanLow.class,
        HasAlgae.class);
  }

  @Override

  public void initialize() {
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.PREP_ALGAE_ZERO);
  }
}
