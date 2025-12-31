// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.second_scoring_element;

import java.util.Set;

import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.subsystems.RobotState;

public class IntakeCoralGroundWithAlgae extends StateCommand {

  public IntakeCoralGroundWithAlgae() {
  }

  @Override
  protected Set<RobotState> getAllowedPreviousStates() {
    return Set.of(RobotState.HAS_ALGAE);
  }

  @Override
  protected RobotState getDesiredState() {
    return RobotState.INTAKE_CORAL_GROUND_WITH_ALGAE;
  }

  @Override
  public void initialize() {
    RobotContainer.subRotors.setCoralIntakeMotorSpeed(constRotorsSpeeds.INTAKE_CORAL_GROUND_SPEED);
  }

  @Override
  public void execute() {
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.INTAKE_CORAL_GROUND);
  }
}
