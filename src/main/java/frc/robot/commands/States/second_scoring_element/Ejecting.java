// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.second_scoring_element;

import java.util.Set;

import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.subsystems.RobotState;

public class Ejecting extends StateCommand {

  public Ejecting() {
  }

  @Override
  protected Set<RobotState> getAllowedPreviousStates() {
    return Set.of(
        RobotState.HAS_CORAL,
        RobotState.HAS_ALGAE,
        RobotState.HAS_CORAL_AND_ALGAE);
  }

  @Override
  protected RobotState getDesiredState() {
    return RobotState.EJECTING;
  }

  @Override
  public void initialize() {
    RobotContainer.subRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.EJECTING_GAME_PIECE_SPEED);
    RobotContainer.subRotors.setCoralIntakeMotorSpeed(constRotorsSpeeds.EJECTING_GAME_PIECE_SPEED);
  }

  @Override
  public void execute() {
    // No continuous execution needed
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.subRotors.setHasCoralOverride(false);
    RobotContainer.subRotors.setHasAlgaeOverride(false);
  }
}
