// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Map;
import java.util.Set;
import java.util.EnumMap;
import java.util.EnumSet;

@Logged
public class StateMachine extends SubsystemBase {
  private static StateMachine instance;

  public static RobotState currentRobotState;

  private static final Map<RobotState, Set<RobotState>> allowedTransitions = new EnumMap<>(RobotState.class);
  
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

  public Trigger mapCommand(RobotState state, Command command) {
        return new Trigger(() -> getRobotState() == state).whileTrue(command);
  }

  public void tryState(RobotState desiredState) {
    Set<RobotState> allowed = allowedTransitions.get(desiredState);
    if (allowed != null && allowed.contains(currentRobotState)) {
      currentRobotState = desiredState;
    }
  }
  static {
    allowedTransitions.put(RobotState.NONE, EnumSet.of(
      RobotState.PREP_CLIMB, RobotState.SCORING_CORAL, RobotState.SCORING_ALGAE,
      RobotState.INTAKE_CORAL_GROUND, RobotState.INTAKE_ALGAE_GROUND, RobotState.INTAKE_CORAL_STATION,
      RobotState.CLEAN_HIGH, RobotState.CLEAN_LOW, RobotState.EJECTING, RobotState.INTAKE_CORAL_L1,
      RobotState.SCORING_CORAL_L1
    ));
    allowedTransitions.put(RobotState.PREP_CLIMB, EnumSet.of(
      RobotState.NONE, RobotState.HAS_CORAL, RobotState.HAS_ALGAE, RobotState.HAS_CORAL_AND_ALGAE
    ));
    allowedTransitions.put(RobotState.CLIMBING, EnumSet.of(
      RobotState.PREP_CLIMB
    ));
    allowedTransitions.put(RobotState.PREP_CORAL_ZERO, EnumSet.of(
      RobotState.HAS_CORAL, RobotState.PREP_CORAL_L2, RobotState.PREP_CORAL_L3, RobotState.PREP_CORAL_L4
    ));
    allowedTransitions.put(RobotState.PREP_CORAL_L1, EnumSet.of(
      RobotState.INTAKE_CORAL_L1
    ));
    allowedTransitions.put(RobotState.PREP_CORAL_L2, EnumSet.of(
      RobotState.HAS_CORAL, RobotState.PREP_CORAL_L3, RobotState.PREP_CORAL_L4, RobotState.PREP_CORAL_ZERO
    ));
    allowedTransitions.put(RobotState.PREP_CORAL_L3, EnumSet.of(
      RobotState.HAS_CORAL, RobotState.PREP_CORAL_L2, RobotState.PREP_CORAL_L4, RobotState.PREP_CORAL_ZERO
    ));
    allowedTransitions.put(RobotState.PREP_CORAL_L4, EnumSet.of(
      RobotState.HAS_CORAL, RobotState.PREP_CORAL_L2, RobotState.PREP_CORAL_L3, RobotState.PREP_CORAL_ZERO
    ));
    allowedTransitions.put(RobotState.PREP_CORAL_L2_WITH_ALGAE, EnumSet.of(
      RobotState.HAS_CORAL_AND_ALGAE, RobotState.PREP_CORAL_L3_WITH_ALGAE, RobotState.PREP_CORAL_L4_WITH_ALGAE,
      RobotState.PREP_CORAL_ZERO_WITH_ALGAE, RobotState.PREP_ALGAE_PROCESSOR_WITH_CORAL, RobotState.PREP_ALGAE_NET_WITH_CORAL
    ));
    allowedTransitions.put(RobotState.PREP_CORAL_L3_WITH_ALGAE, EnumSet.of(
      RobotState.HAS_CORAL_AND_ALGAE, RobotState.PREP_CORAL_L2_WITH_ALGAE, RobotState.PREP_CORAL_L4_WITH_ALGAE,
      RobotState.PREP_CORAL_ZERO_WITH_ALGAE, RobotState.PREP_ALGAE_PROCESSOR_WITH_CORAL, RobotState.PREP_ALGAE_NET_WITH_CORAL
    ));
    allowedTransitions.put(RobotState.PREP_CORAL_L4_WITH_ALGAE, EnumSet.of(
      RobotState.HAS_CORAL_AND_ALGAE, RobotState.PREP_CORAL_L2_WITH_ALGAE, RobotState.PREP_CORAL_L3_WITH_ALGAE,
      RobotState.PREP_CORAL_ZERO_WITH_ALGAE, RobotState.PREP_ALGAE_PROCESSOR_WITH_CORAL, RobotState.PREP_ALGAE_NET_WITH_CORAL
    ));
    allowedTransitions.put(RobotState.PREP_CORAL_ZERO_WITH_ALGAE, EnumSet.of(
      RobotState.HAS_CORAL_AND_ALGAE, RobotState.PREP_CORAL_L2_WITH_ALGAE, RobotState.PREP_CORAL_L3_WITH_ALGAE,
      RobotState.PREP_CORAL_L4_WITH_ALGAE, RobotState.PREP_ALGAE_PROCESSOR_WITH_CORAL, RobotState.PREP_ALGAE_NET_WITH_CORAL
    ));
    allowedTransitions.put(RobotState.PREP_ALGAE_NET, EnumSet.of(
      RobotState.HAS_ALGAE, RobotState.PREP_ALGAE_PROCESSOR, RobotState.PREP_ALGAE_ZERO
    ));
    allowedTransitions.put(RobotState.PREP_ALGAE_PROCESSOR, EnumSet.of(
      RobotState.HAS_ALGAE, RobotState.PREP_ALGAE_ZERO, RobotState.PREP_ALGAE_NET
    ));
    allowedTransitions.put(RobotState.PREP_ALGAE_ZERO, EnumSet.of(
      RobotState.HAS_ALGAE, RobotState.PREP_ALGAE_NET, RobotState.PREP_ALGAE_PROCESSOR
    ));
    allowedTransitions.put(RobotState.PREP_ALGAE_NET_WITH_CORAL, EnumSet.of(
      RobotState.HAS_CORAL_AND_ALGAE, RobotState.PREP_ALGAE_PROCESSOR_WITH_CORAL, RobotState.PREP_CORAL_L2_WITH_ALGAE,
      RobotState.PREP_CORAL_L3_WITH_ALGAE, RobotState.PREP_CORAL_L4_WITH_ALGAE, RobotState.PREP_CORAL_ZERO_WITH_ALGAE
    ));
    allowedTransitions.put(RobotState.PREP_ALGAE_PROCESSOR_WITH_CORAL, EnumSet.of(
      RobotState.HAS_CORAL_AND_ALGAE, RobotState.PREP_ALGAE_NET_WITH_CORAL, RobotState.PREP_CORAL_L2_WITH_ALGAE,
      RobotState.PREP_CORAL_L3_WITH_ALGAE, RobotState.PREP_CORAL_L4_WITH_ALGAE, RobotState.PREP_CORAL_ZERO_WITH_ALGAE
    ));
    allowedTransitions.put(RobotState.HAS_CORAL, EnumSet.of(
      RobotState.INTAKE_CORAL_GROUND, RobotState.INTAKE_CORAL_STATION, RobotState.SCORING_ALGAE_WITH_CORAL,
      RobotState.INTAKE_ALGAE_GROUND_WITH_CORAL
    ));
    allowedTransitions.put(RobotState.HAS_ALGAE, EnumSet.of(
      RobotState.CLEAN_HIGH, RobotState.CLEAN_LOW, RobotState.INTAKE_ALGAE_GROUND, RobotState.SCORING_CORAL_WITH_ALGAE,
      RobotState.INTAKE_CORAL_GROUND_WITH_ALGAE, RobotState.INTAKE_CORAL_STATION_WITH_ALGAE
    ));
    allowedTransitions.put(RobotState.HAS_CORAL_AND_ALGAE, EnumSet.of(
      RobotState.INTAKE_ALGAE_GROUND_WITH_CORAL, RobotState.INTAKE_CORAL_GROUND_WITH_ALGAE,
      RobotState.INTAKE_CORAL_STATION_WITH_ALGAE, RobotState.CLEAN_HIGH_WITH_CORAL, RobotState.CLEAN_LOW_WITH_CORAL
    ));
    allowedTransitions.put(RobotState.SCORING_CORAL, EnumSet.of(
      RobotState.PREP_CORAL_L2, RobotState.PREP_CORAL_L3, RobotState.PREP_CORAL_L4
    ));
    allowedTransitions.put(RobotState.SCORING_ALGAE, EnumSet.of(
      RobotState.PREP_ALGAE_NET, RobotState.PREP_ALGAE_PROCESSOR, RobotState.PREP_ALGAE_ZERO
    ));
    allowedTransitions.put(RobotState.SCORING_CORAL_L1, EnumSet.of(
      RobotState.PREP_CORAL_L1
    ));
    allowedTransitions.put(RobotState.CLEAN_HIGH, EnumSet.of(
      RobotState.CLEAN_LOW, RobotState.INTAKE_ALGAE_GROUND, RobotState.NONE
    ));
    allowedTransitions.put(RobotState.CLEAN_LOW, EnumSet.of(
      RobotState.NONE, RobotState.INTAKE_ALGAE_GROUND, RobotState.CLEAN_HIGH
    ));
    allowedTransitions.put(RobotState.INTAKE_CORAL_STATION, EnumSet.of(
      RobotState.NONE, RobotState.INTAKE_CORAL_GROUND
    ));
    allowedTransitions.put(RobotState.INTAKE_ALGAE_GROUND, EnumSet.of(
      RobotState.NONE, RobotState.CLEAN_HIGH, RobotState.CLEAN_LOW
    ));
    allowedTransitions.put(RobotState.INTAKE_CORAL_L1, EnumSet.of(
      RobotState.NONE, RobotState.INTAKE_CORAL_STATION, RobotState.INTAKE_CORAL_GROUND
    ));
    allowedTransitions.put(RobotState.EJECTING, EnumSet.of(
      RobotState.NONE, RobotState.HAS_ALGAE, RobotState.HAS_CORAL, RobotState.HAS_CORAL_AND_ALGAE,
      RobotState.PREP_ALGAE_NET, RobotState.PREP_ALGAE_NET_WITH_CORAL, RobotState.PREP_ALGAE_PROCESSOR,
      RobotState.PREP_ALGAE_PROCESSOR_WITH_CORAL, RobotState.PREP_ALGAE_ZERO, RobotState.PREP_CORAL_ZERO_WITH_ALGAE,
      RobotState.PREP_CORAL_L2, RobotState.PREP_CORAL_L3, RobotState.PREP_CORAL_L4,
      RobotState.PREP_CORAL_L2_WITH_ALGAE, RobotState.PREP_CORAL_L3_WITH_ALGAE, RobotState.PREP_CORAL_L4_WITH_ALGAE,
      RobotState.PREP_CORAL_ZERO
    ));
    allowedTransitions.put(RobotState.SCORING_ALGAE_WITH_CORAL, EnumSet.of(
      RobotState.PREP_ALGAE_NET_WITH_CORAL, RobotState.PREP_ALGAE_PROCESSOR_WITH_CORAL, RobotState.PREP_CORAL_ZERO_WITH_ALGAE
    ));
    allowedTransitions.put(RobotState.SCORING_CORAL_WITH_ALGAE, EnumSet.of(
      RobotState.PREP_CORAL_L2_WITH_ALGAE, RobotState.PREP_CORAL_L3_WITH_ALGAE, RobotState.PREP_CORAL_L4_WITH_ALGAE,
      RobotState.PREP_CORAL_ZERO_WITH_ALGAE
    ));
    allowedTransitions.put(RobotState.CLEAN_HIGH_WITH_CORAL, EnumSet.of(
      RobotState.CLEAN_LOW_WITH_CORAL, RobotState.PREP_CORAL_L2, RobotState.PREP_CORAL_L3, RobotState.PREP_CORAL_L4,
      RobotState.HAS_CORAL, RobotState.INTAKE_ALGAE_GROUND_WITH_CORAL
    ));
    allowedTransitions.put(RobotState.CLEAN_LOW_WITH_CORAL, EnumSet.of(
      RobotState.HAS_CORAL, RobotState.PREP_CORAL_L2, RobotState.PREP_CORAL_L3, RobotState.PREP_CORAL_L4,
      RobotState.CLEAN_HIGH_WITH_CORAL, RobotState.INTAKE_ALGAE_GROUND_WITH_CORAL
    ));
    allowedTransitions.put(RobotState.INTAKE_CORAL_GROUND, EnumSet.of(
      RobotState.NONE, RobotState.INTAKE_CORAL_STATION
    ));
    allowedTransitions.put(RobotState.INTAKE_CORAL_GROUND_WITH_ALGAE, EnumSet.noneOf(RobotState.class));
    allowedTransitions.put(RobotState.INTAKE_ALGAE_GROUND_WITH_CORAL, EnumSet.noneOf(RobotState.class));
    allowedTransitions.put(RobotState.INTAKE_CORAL_STATION_WITH_ALGAE, EnumSet.noneOf(RobotState.class));
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

}
