// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class StateMachine extends SubsystemBase {
  private static StateMachine instance;

  public static RobotState currentRobotState;

  
  private StateMachine() {
    currentRobotState = RobotState.NONE;
  }

  public static StateMachine getInstance() {
    if (instance == null) {
      instance = new StateMachine();
    }
    return instance;
  }

  public void setRobotState(RobotState robotState) {
    currentRobotState = robotState;
  }

  public RobotState getRobotState() {
    return currentRobotState;
  }

  public boolean inCleaningState() {
    RobotState[] goToHasBothStates = { RobotState.CLEAN_HIGH, RobotState.CLEAN_LOW,
        RobotState.CLEAN_HIGH_WITH_CORAL,
        RobotState.CLEAN_LOW_WITH_CORAL };
    for (RobotState state : goToHasBothStates) {
      if (currentRobotState == state) {
        return true;
      }
    }
    return false;
  }

  public void tryState(RobotState desiredState) {
    switch (desiredState) {
      case NONE:
        switch (currentRobotState) {
          case PREP_CLIMB:
          case SCORING_CORAL:
          case SCORING_ALGAE:
          case INTAKE_CORAL_GROUND:
          case INTAKE_ALGAE_GROUND:
          case INTAKE_CORAL_STATION:
          case CLEAN_HIGH:
          case CLEAN_LOW:
          case EJECTING:
          case INTAKE_CORAL_L1:
          case SCORING_CORAL_L1:
            currentRobotState = desiredState;
            return;
        }
        break;
      // climbing states
      case PREP_CLIMB:
        switch (currentRobotState) {
          case NONE:
          case HAS_CORAL:
          case HAS_ALGAE:
          case HAS_CORAL_AND_ALGAE:
            currentRobotState = desiredState;
            return;
        }
        break;
      case CLIMBING:
        switch (currentRobotState) {
          case PREP_CLIMB:
            currentRobotState = desiredState;
            return;
        }
        break;
      // Prep Coral only
      case PREP_CORAL_ZERO:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
            currentRobotState = desiredState;
            return;
        }
        break;
      case PREP_CORAL_L1:
        switch (currentRobotState) {
          case INTAKE_CORAL_L1:
            currentRobotState = desiredState;
            return;
        }
        break;
      case PREP_CORAL_L2:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
          case PREP_CORAL_ZERO:
            currentRobotState = desiredState;
            return;
        }
        break;
      case PREP_CORAL_L3:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L2:
          case PREP_CORAL_L4:
          case PREP_CORAL_ZERO:
            currentRobotState = desiredState;
            return;
        }
        break;
      case PREP_CORAL_L4:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_ZERO:
            currentRobotState = desiredState;
            return;
        }
        break;
      // prep Coral with Algae
      case PREP_CORAL_L2_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_ALGAE_PROCESSOR_WITH_CORAL:
          case PREP_ALGAE_NET_WITH_CORAL:
            currentRobotState = desiredState;
            return;
        }
        break;
      case PREP_CORAL_L3_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_ALGAE_PROCESSOR_WITH_CORAL:
          case PREP_ALGAE_NET_WITH_CORAL:
            currentRobotState = desiredState;
            return;
        }
        break;
      case PREP_CORAL_L4_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_ALGAE_PROCESSOR_WITH_CORAL:
          case PREP_ALGAE_NET_WITH_CORAL:
            currentRobotState = desiredState;
            return;
        }
        break;
      case PREP_CORAL_ZERO_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_ALGAE_PROCESSOR_WITH_CORAL:
          case PREP_ALGAE_NET_WITH_CORAL:
            currentRobotState = desiredState;
            return;
        }
        break;
      // prep Algae only
      case PREP_ALGAE_NET:
        switch (currentRobotState) {
          case HAS_ALGAE:
          case PREP_ALGAE_PROCESSOR:
          case PREP_ALGAE_ZERO:
            currentRobotState = desiredState;
            return;
        }
        break;
      case PREP_ALGAE_PROCESSOR:
        switch (currentRobotState) {
          case HAS_ALGAE:
          case PREP_ALGAE_ZERO:
          case PREP_ALGAE_NET:
            currentRobotState = desiredState;
            return;
        }
        break;
      case PREP_ALGAE_ZERO:
        switch (currentRobotState) {
          case HAS_ALGAE:
          case PREP_ALGAE_NET:
          case PREP_ALGAE_PROCESSOR:
            currentRobotState = desiredState;
            return;
        }
        break;
      // prep Algae with Coral
      case PREP_ALGAE_NET_WITH_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_ALGAE_PROCESSOR_WITH_CORAL:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
            currentRobotState = desiredState;
            return;
        }
        break;
      case PREP_ALGAE_PROCESSOR_WITH_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_ALGAE_NET_WITH_CORAL:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
            currentRobotState = desiredState;
            return;
        }
        break;
      // holding 1 game piece
      case HAS_CORAL:
        switch (currentRobotState) {
          case INTAKE_CORAL_GROUND:
          case INTAKE_CORAL_STATION:
          case SCORING_ALGAE_WITH_CORAL:
          case INTAKE_ALGAE_GROUND_WITH_CORAL:
            currentRobotState = desiredState;
            return;
        }
        break;
      case HAS_ALGAE:
        switch (currentRobotState) {
          case CLEAN_HIGH:
          case CLEAN_LOW:
          case INTAKE_ALGAE_GROUND:
          case SCORING_CORAL_WITH_ALGAE:
          case INTAKE_CORAL_GROUND_WITH_ALGAE:
          case INTAKE_CORAL_STATION_WITH_ALGAE:
            currentRobotState = desiredState;
            return;
        }
        break;
      // holding 2 game pieces
      case HAS_CORAL_AND_ALGAE:
        switch (currentRobotState) {
          case INTAKE_ALGAE_GROUND_WITH_CORAL:
          case INTAKE_CORAL_GROUND_WITH_ALGAE:
          case INTAKE_CORAL_STATION_WITH_ALGAE:
          case CLEAN_HIGH_WITH_CORAL:
          case CLEAN_LOW_WITH_CORAL:
            currentRobotState = desiredState;
            return;
        }
        break;
      // manipulating 1 game piece
      case SCORING_CORAL:
        switch (currentRobotState) {
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
            currentRobotState = desiredState;
            return;
        }
        break;
      case SCORING_ALGAE:
        switch (currentRobotState) {
          case PREP_ALGAE_NET:
          case PREP_ALGAE_PROCESSOR:
          case PREP_ALGAE_ZERO:
            currentRobotState = desiredState;
            return;
        }
        break;
      case SCORING_CORAL_L1:
        switch (currentRobotState) {
          case PREP_CORAL_L1:
            currentRobotState = desiredState;
            return;
        }
        break;
      case CLEAN_HIGH:
        switch (currentRobotState) {
          case CLEAN_LOW:
          case INTAKE_ALGAE_GROUND:
          case NONE:
            currentRobotState = desiredState;
            return;
        }
        break;
      case CLEAN_LOW:
        switch (currentRobotState) {
          case NONE:
          case INTAKE_ALGAE_GROUND:
          case CLEAN_HIGH:
            currentRobotState = desiredState;
            return;
        }
        break;
      case INTAKE_CORAL_STATION:
        switch (currentRobotState) {
          case NONE:
          case INTAKE_CORAL_GROUND:
            currentRobotState = desiredState;
            return;
        }
        break;
      case INTAKE_ALGAE_GROUND:
        switch (currentRobotState) {
          case NONE:
          case CLEAN_HIGH:
          case CLEAN_LOW:
            currentRobotState = desiredState;
            return;
        }
        break;
      case INTAKE_CORAL_L1:
        switch (currentRobotState) {
          case NONE:
          case INTAKE_CORAL_STATION:
          case INTAKE_CORAL_GROUND:
            currentRobotState = desiredState;
            return;
        }
        break;
      // manipulating 2 game pieces
      case EJECTING:
        switch (currentRobotState) {
          case NONE:
          case HAS_ALGAE:
          case HAS_CORAL:
          case HAS_CORAL_AND_ALGAE:
          case PREP_ALGAE_NET:
          case PREP_ALGAE_NET_WITH_CORAL:
          case PREP_ALGAE_PROCESSOR:
          case PREP_ALGAE_PROCESSOR_WITH_CORAL:
          case PREP_ALGAE_ZERO:
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_CORAL_ZERO:
            currentRobotState = desiredState;
            return;
        }
        break;
      case SCORING_ALGAE_WITH_CORAL:
        switch (currentRobotState) {
          case PREP_ALGAE_NET_WITH_CORAL:
          case PREP_ALGAE_PROCESSOR_WITH_CORAL:
          case PREP_CORAL_ZERO_WITH_ALGAE:
            currentRobotState = desiredState;
            return;
        }
        break;
      case SCORING_CORAL_WITH_ALGAE:
        switch (currentRobotState) {
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
            currentRobotState = desiredState;
            return;
        }
        break;
      case CLEAN_HIGH_WITH_CORAL:
        switch (currentRobotState) {
          case CLEAN_LOW_WITH_CORAL:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
          case HAS_CORAL:
          case INTAKE_ALGAE_GROUND_WITH_CORAL:
            currentRobotState = desiredState;
            return;
        }
        break;
      case CLEAN_LOW_WITH_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
          case CLEAN_HIGH_WITH_CORAL:
          case INTAKE_ALGAE_GROUND_WITH_CORAL:
            currentRobotState = desiredState;
            return;
        }
        break;
      case INTAKE_CORAL_GROUND:
        switch (currentRobotState) {
          case NONE:
          case INTAKE_CORAL_STATION:
            currentRobotState = desiredState;
            return;
        }
        break;
    }
    return;
  }

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
