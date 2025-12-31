// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.first_scoring_element;

import java.util.Set;

import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.subsystems.RobotState;

public class IntakeCoralL1 extends StateCommand {

  public IntakeCoralL1() {
  }

  @Override
  protected Set<RobotState> getAllowedPreviousStates() {
    return Set.of(RobotState.NONE, RobotState.HAS_CORAL);
  }

  @Override
  protected RobotState getDesiredState() {
    return RobotState.INTAKE_CORAL_L1;
  }

  @Override
  public void initialize() {
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.INTAKE_CORAL_L1);
  }

  @Override
  public void execute() {
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.INTAKE_CORAL_L1);
    if (!RobotContainer.subRotors.seeL1Coral()) {
      RobotContainer.subRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.INTAKE_L1_SPEED);
    } else if (RobotContainer.subRotors.seeL1Coral() && !RobotContainer.subRotors.hasL1Coral()) {
      RobotContainer.subRotors.indexL1Coral(constRotorsSpeeds.INDEX_L1_SPEED);
    }
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.subRotors.hasL1Coral();
  }
}
