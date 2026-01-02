// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.scoring;

import java.util.Set;

import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.RobotContainer;
import frc.robot.commands.StatefulCommand;
import frc.robot.commands.States.prep_algae.PrepNet;
import frc.robot.commands.States.prep_algae.PrepNetWithCoral;
import frc.robot.commands.States.prep_algae.PrepProcessor;
import frc.robot.commands.States.prep_algae.PrepProcessorWithCoral;

public class ScoringAlgaeWithCoral extends StatefulCommand {

  public ScoringAlgaeWithCoral() {
  }

  @Override
  protected Set<Class<? extends StatefulCommand>> getAllowedPreviousStates() {
    return Set.of(
        PrepNetWithCoral.class,
        PrepProcessorWithCoral.class);
  }

  @Override

  public void initialize() {
    if (StatefulCommand.getCurrentState() == PrepNet.class) {
      RobotContainer.subRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.SCORE_ALGAE_NET_SPEED);
    } else if (StatefulCommand.getCurrentState() == PrepProcessor.class) {
      RobotContainer.subRotors.setAlgaeIntakeMotorSpeed(constRotorsSpeeds.SCORE_ALGAE_PROCESSOR_SPEED);
    }
  }

  @Override
  public void execute() {
    // No continuous execution needed
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.subRotors.setHasAlgaeOverride(false);
  }
}
