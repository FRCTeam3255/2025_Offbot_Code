// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.climbing;

import java.util.Set;

import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.Elastic;
import frc.robot.RobotContainer;
import frc.robot.commands.StatefulCommand;
import frc.robot.commands.States.None;
import frc.robot.commands.States.hold_scoring_elements.HasAlgae;
import frc.robot.commands.States.hold_scoring_elements.HasCoral;
import frc.robot.commands.States.hold_scoring_elements.HasCoralAndAlgae;

public class PrepClimb extends StatefulCommand {

  public PrepClimb() {
  }

  @Override
  protected Set<Class<? extends StatefulCommand>> getAllowedPreviousStates() {
    return Set.of(None.class, HasCoral.class,
        HasAlgae.class,
        HasCoralAndAlgae.class,
        Climbing.class);
  }

  @Override

  public void initialize() {
    RobotContainer.subRotors.setClimberMotorPercentOutput(constRotorsSpeeds.CLIMBER_MOTOR_PERCENT_OUTPUT);
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.PREP_CLIMB);
    Elastic.selectTab("Climbing");
  }
}
