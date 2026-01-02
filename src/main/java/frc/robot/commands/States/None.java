// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States;

import java.util.Set;

import frc.robot.Constants.constMechanismPositions;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.commands.States.climbing.Climbing;
import frc.robot.commands.States.climbing.PrepClimb;
import frc.robot.commands.States.first_scoring_element.CleanHigh;
import frc.robot.commands.States.first_scoring_element.CleanLow;
import frc.robot.commands.States.first_scoring_element.IntakeAlgaeGround;
import frc.robot.commands.States.first_scoring_element.IntakeCoralGround;
import frc.robot.commands.States.first_scoring_element.IntakeCoralL1;
import frc.robot.commands.States.first_scoring_element.IntakeCoralStation;
import frc.robot.commands.States.scoring.ScoringAlgae;
import frc.robot.commands.States.scoring.ScoringCoral;
import frc.robot.commands.States.scoring.ScoringL1Coral;
import frc.robot.commands.States.second_scoring_element.Ejecting;

public class None extends StateCommand {

  public None() {
  }

  @Override
  protected Set<Class<? extends StateCommand>> getAllowedPreviousStates() {
    return Set.of(PrepClimb.class, ScoringCoral.class, ScoringAlgae.class,
        IntakeCoralGround.class, IntakeAlgaeGround.class,
        IntakeCoralStation.class, CleanHigh.class, CleanLow.class,
        Ejecting.class, IntakeCoralL1.class, ScoringL1Coral.class,
        Climbing.class);
  }

  @Override

  public void initialize() {
    RobotContainer.subRotors.setAlgaeIntakeMotorSpeed(0);
    RobotContainer.subRotors.setCoralIntakeMotorSpeed(0);
    RobotContainer.subRotors.setClimberMotorPercentOutput(0);
    RobotContainer.subMotion.setAllPosition(constMechanismPositions.NONE);
  }
}
