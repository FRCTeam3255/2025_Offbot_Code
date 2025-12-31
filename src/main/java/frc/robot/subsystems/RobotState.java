// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

public enum RobotState {
  NONE,
  // climbing states
  PREP_CLIMB,
  CLIMBING,
  // Prep Coral only
  PREP_CORAL_ZERO,
  PREP_CORAL_L1,
  PREP_CORAL_L2,
  PREP_CORAL_L3,
  PREP_CORAL_L4,
  // prep Coral with Algae
  PREP_CORAL_L2_WITH_ALGAE,
  PREP_CORAL_L3_WITH_ALGAE,
  PREP_CORAL_L4_WITH_ALGAE,
  PREP_CORAL_ZERO_WITH_ALGAE,
  // prep Algae only
  PREP_ALGAE_NET,
  PREP_ALGAE_PROCESSOR,
  PREP_ALGAE_ZERO,
  // prep Algae with Coral
  PREP_ALGAE_NET_WITH_CORAL,
  PREP_ALGAE_PROCESSOR_WITH_CORAL,
  // holding 1 game piece
  HAS_CORAL,
  HAS_ALGAE,
  // holding 2 game pieces
  HAS_CORAL_AND_ALGAE,
  // manipulating 1 game piece
  SCORING_CORAL,
  SCORING_ALGAE,
  SCORING_CORAL_L1,
  CLEAN_HIGH,
  CLEAN_LOW,
  INTAKE_CORAL_STATION,
  INTAKE_ALGAE_GROUND,
  INTAKE_CORAL_L1,
  // manipulating 2 game pieces
  EJECTING, // we are planning on ejecting both game pieces at the same time
  SCORING_ALGAE_WITH_CORAL,
  SCORING_CORAL_WITH_ALGAE,
  CLEAN_HIGH_WITH_CORAL,
  CLEAN_LOW_WITH_CORAL,
  INTAKE_CORAL_GROUND,
  INTAKE_CORAL_GROUND_WITH_ALGAE,
  INTAKE_ALGAE_GROUND_WITH_CORAL,
  INTAKE_CORAL_STATION_WITH_ALGAE,
}
