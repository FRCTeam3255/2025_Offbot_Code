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
import frc.robot.commands.States.hold_scoring_elements.HasCoral;

public class IntakeCoralGround extends StateCommand {

  public IntakeCoralGround() {
  }

  @Override
  protected Set<Class<? extends StateCommand>> getAllowedPreviousStates() {
    return Set.of(None.class, HasCoral.class);
  }

  @Override

  public void initialize() {
    RobotContainer.subRotors.setCoralIntakeMotorSpeed(constRotorsSpeeds.INTAKE_CORAL_GROUND_SPEED);
    RobotContainer.subRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.INTAKE_CORAL_ALGAE_SPEED);
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.INTAKE_CORAL_GROUND);
  }

  @Override
  public void execute() {
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.INTAKE_CORAL_GROUND);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.subRotors.setAllIntake(0);
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.subRotors.hasCoral();
  }
}
