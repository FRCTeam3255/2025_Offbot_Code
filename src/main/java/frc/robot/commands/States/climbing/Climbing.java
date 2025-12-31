// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.climbing;

import java.util.Set;

import frc.robot.Constants.constMechanismPositions;
import frc.robot.Elastic;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.subsystems.RobotState;

public class Climbing extends StateCommand {

  public Climbing() {
  }

  @Override
  protected Set<RobotState> getAllowedPreviousStates() {
    return Set.of(RobotState.PREP_CLIMB);
  }

  @Override
  protected RobotState getDesiredState() {
    return RobotState.CLIMBING;
  }

  @Override
  public void initialize() {
    Elastic.selectTab("Climbing");
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.CLIMBED);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.LATCHED);
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.subMotion.arePositionsAtSetPoint(constMechanismPositions.CLIMBED);
  }
}
