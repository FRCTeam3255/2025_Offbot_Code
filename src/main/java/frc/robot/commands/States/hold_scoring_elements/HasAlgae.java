// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.hold_scoring_elements;

import java.util.Set;

import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.commands.States.first_scoring_element.CleanHigh;
import frc.robot.commands.States.first_scoring_element.CleanLow;
import frc.robot.commands.States.first_scoring_element.IntakeAlgaeGround;
import frc.robot.commands.States.second_scoring_element.CleanHighWithCoral;
import frc.robot.commands.States.second_scoring_element.CleanLowWithCoral;
import frc.robot.commands.States.second_scoring_element.IntakeAlgaeGroundWithCoral;

public class HasAlgae extends StateCommand {

  public HasAlgae() {
  }

  @Override
  protected Set<Class<? extends StateCommand>> getAllowedPreviousStates() {
    return Set.of(
        IntakeAlgaeGround.class,
        IntakeAlgaeGroundWithCoral.class,
        CleanHigh.class,
        CleanLow.class,
        CleanHighWithCoral.class,
        CleanLowWithCoral.class);
  }

  @Override

  public void initialize() {
    if (RobotContainer.getRobotState() == IntakeAlgaeGround.class ||
        RobotContainer.getRobotState() == IntakeAlgaeGroundWithCoral.class) {
      RobotContainer.subRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.ALGAE_HOLD_SPEED);
    } else if (RobotContainer.getRobotState() == CleanHigh.class ||
        RobotContainer.getRobotState() == CleanLow.class ||
        RobotContainer.getRobotState() == CleanHighWithCoral.class ||
        RobotContainer.getRobotState() == CleanLowWithCoral.class) {
      RobotContainer.subRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.ALGAE_HARD_HOLD_SPEED);
    }
    RobotContainer.subRotors.setHasAlgaeOverride(true);
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.HAS_SCORING_ELEMENTS);
  }
}
