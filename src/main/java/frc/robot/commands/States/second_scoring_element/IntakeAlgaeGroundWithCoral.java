// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.second_scoring_element;

import java.util.Set;

import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.RobotContainer;
import frc.robot.commands.StatefulCommand;
import frc.robot.commands.States.hold_scoring_elements.HasCoral;

public class IntakeAlgaeGroundWithCoral extends StatefulCommand {

  public IntakeAlgaeGroundWithCoral() {
  }

  @Override
  protected Set<Class<? extends StatefulCommand>> getAllowedPreviousStates() {
    return Set.of(HasCoral.class);
  }

  @Override

  public void initialize() {
    RobotContainer.subRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.INTAKE_ALGAE_SPEED);
  }

  @Override
  public void execute() {
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.INTAKE_ALGAE_GROUND);
  }
}
