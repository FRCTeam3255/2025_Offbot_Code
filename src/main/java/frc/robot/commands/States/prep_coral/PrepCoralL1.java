// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.prep_coral;

import java.util.Set;

import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.commands.States.first_scoring_element.IntakeCoralL1;
import frc.robot.commands.States.None;
import frc.robot.commands.States.first_scoring_element.CleanHigh;
import frc.robot.commands.States.first_scoring_element.CleanLow;
import frc.robot.commands.States.first_scoring_element.IntakeCoralGround;
import frc.robot.commands.States.first_scoring_element.IntakeCoralStation;
import frc.robot.commands.States.hold_scoring_elements.HasCoral;

public class PrepCoralL1 extends StateCommand {
  @Override
  protected Set<Class<? extends StateCommand>> getAllowedPreviousStates() {
    return Set.of(
        HasCoral.class,
        None.class,
        CleanHigh.class,
        CleanLow.class,
        IntakeCoralGround.class,
        IntakeCoralStation.class,
        IntakeCoralL1.class);
  }

  @Override

  public void initialize() {
    RobotContainer.subRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.L1_CORAL_HOLD_SPEED);
    RobotContainer.subRotors.setCoralIntakeMotorSpeed(constRotorsSpeeds.CORAL_L1_CORAL_HOLD_SPEED);
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.PREP_CORAL_L1);
  }
}
