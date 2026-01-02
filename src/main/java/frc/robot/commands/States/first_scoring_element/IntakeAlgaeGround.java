// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.first_scoring_element;

import java.util.Set;

import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.commands.States.None;
import frc.robot.commands.States.hold_scoring_elements.HasAlgae;

public class IntakeAlgaeGround extends StateCommand {

  public IntakeAlgaeGround() {
  }

  @Override
  protected Set<Class<? extends StateCommand>> getAllowedPreviousStates() {
    return Set.of(None.class, HasAlgae.class);
  }

  @Override

  public void initialize() {
    RobotContainer.subRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.INTAKE_ALGAE_SPEED);
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.INTAKE_ALGAE_GROUND);
  }

  @Override
  public void execute() {
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.INTAKE_ALGAE_GROUND);
  }
}
