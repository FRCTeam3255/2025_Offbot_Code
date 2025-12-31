// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.prep_coral;

import java.util.Set;

import frc.robot.Constants.MechanismPositionGroup;
import frc.robot.Constants.constMechanismPositions;
import frc.robot.Field;
import frc.robot.RobotContainer;
import frc.robot.commands.StateCommand;
import frc.robot.subsystems.RobotState;

public class PrepCoralWithAlgae extends StateCommand {
  MechanismPositionGroup prepL2;
  MechanismPositionGroup prepL3;
  MechanismPositionGroup prepL4;
  int targetLevel;

  public PrepCoralWithAlgae(int level) {
    targetLevel = level;
  }

  @Override
  protected Set<RobotState> getAllowedPreviousStates() {
    return Set.of(
        RobotState.HAS_ALGAE,
        RobotState.INTAKE_ALGAE_GROUND,
        RobotState.CLEAN_HIGH,
        RobotState.CLEAN_LOW);
  }

  @Override
  protected RobotState getDesiredState() {
    return targetLevel == 0 ? RobotState.PREP_CORAL_ZERO_WITH_ALGAE
        : targetLevel == 2 ? RobotState.PREP_CORAL_L2_WITH_ALGAE
            : targetLevel == 3 ? RobotState.PREP_CORAL_L3_WITH_ALGAE : RobotState.PREP_CORAL_L4_WITH_ALGAE;
  }

  @Override
  public void initialize() {
    if (RobotContainer.subDrivetrain.isActionBackwards(
        Field.FieldElementGroups.REEF_POSES.getAll()) == true) {
      prepL2 = constMechanismPositions.PREP_CORAL_L2_BACKWARDS;
      prepL3 = constMechanismPositions.PREP_CORAL_L3_BACKWARDS;
      prepL4 = constMechanismPositions.PREP_CORAL_L4_BACKWARDS;
    } else {
      prepL2 = constMechanismPositions.PREP_CORAL_L2_FORWARDS;
      prepL3 = constMechanismPositions.PREP_CORAL_L3_FORWARDS;
      prepL4 = constMechanismPositions.PREP_CORAL_L4_FORWARDS;
    }

    if (targetLevel == 0) {
      RobotContainer.subMotion.setAllPosition(constMechanismPositions.PREP_CORAL_ZERO);
    } else if (targetLevel == 2) {
      RobotContainer.subMotion.setAllPosition(prepL2);
    } else if (targetLevel == 3) {
      RobotContainer.subMotion.setAllPosition(prepL3);
    } else if (targetLevel == 4) {
      RobotContainer.subMotion.setAllPosition(prepL4);
    }
  }
}
