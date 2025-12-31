// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.climbing;

import java.util.Set;

import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.Elastic;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.subsystems.RobotState;

public class PrepClimb extends StateCommand {

  public PrepClimb() {
  }

  @Override
  protected Set<RobotState> getAllowedPreviousStates() {
    return Set.of(RobotState.NONE, RobotState.HAS_CORAL, RobotState.HAS_ALGAE,
        RobotState.HAS_CORAL_AND_ALGAE, RobotState.CLIMBING);
  }

  @Override
  protected RobotState getDesiredState() {
    return RobotState.PREP_CLIMB;
  }

  @Override
  public void initialize() {
    RobotContainer.subRotors.setClimberMotorPercentOutput(constRotorsSpeeds.CLIMBER_MOTOR_PERCENT_OUTPUT);
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.PREP_CLIMB);
    Elastic.selectTab("Climbing");
  }
}
