// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.*;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.States.*;
import frc.robot.subsystems.*;

@Logged
public class StateMachine extends SubsystemBase {
  public static DriverState currentDriverState;
  public static RobotState currentRobotState;
  @NotLogged
  Drivetrain subDrivetrain;
  @NotLogged
  StateMachine subStateMachine = this;

  /** Creates a new StateMachine. */
  public StateMachine(Drivetrain subDrivetrain) {
    currentRobotState = RobotState.NONE;
    currentDriverState = DriverState.MANUAL;

    this.subDrivetrain = subDrivetrain;
  }

  public void setRobotState(RobotState robotState) {
    currentRobotState = robotState;
  }

  public RobotState getRobotState() {
    return currentRobotState;
  }

  public DriverState getDriverState() {
    return currentDriverState;
  }

  public void setDriverState(DriverState driverState) {
    currentDriverState = driverState;
  }

  public Command tryState(RobotState desiredState) {
    switch (desiredState) {
      case NONE:
        switch (currentRobotState) {
          case NONE:
            return new None(subStateMachine);
        }
        break;
      case PREP_CLIMB:
        switch (currentRobotState) {
          case PREP_CLIMB:
            return new PrepClimb(subStateMachine);
        }
        break;
      case CLIMBINGPREP_CORAL_L1:
        switch (currentRobotState) {
          case CLIMBINGPREP_CORAL_L1:
            return new ClimbingprepCoralL1(subStateMachine);
        }
        break;
      case PREP_CORAL_L2:
        switch (currentRobotState) {
          case PREP_CORAL_L2:
            return new PrepCoralL2(subStateMachine);
        }
        break;
      case PREP_CORAL_L3:
        switch (currentRobotState) {
          case PREP_CORAL_L3:
            return new PrepCoralL3(subStateMachine);
        }
        break;
      case PREP_CORAL_L4:
        switch (currentRobotState) {
          case PREP_CORAL_L4:
            return new PrepCoralL4(subStateMachine);
        }
        break;
      case PREP_CORAL_ZERO:
        switch (currentRobotState) {
          case PREP_CORAL_ZERO:
            return new PrepCoralZero(subStateMachine);
        }
        break;
      case PREP_CORAL_WITH_ALGAE_L1:
        switch (currentRobotState) {
          case PREP_CORAL_WITH_ALGAE_L1:
            return new PrepCoralWithAlgaeL1(subStateMachine);
        }
        break;
      case PREP_CORAL_WITH_ALGAE_L2:
        switch (currentRobotState) {
          case PREP_CORAL_WITH_ALGAE_L2:
            return new PrepCoralWithAlgaeL2(subStateMachine);
        }
        break;
      case PREP_CORAL_WITH_ALGAE_L3:
        switch (currentRobotState) {
          case PREP_CORAL_WITH_ALGAE_L3:
            return new PrepCoralWithAlgaeL3(subStateMachine);
        }
        break;
      case PREP_CORAL_WITH_ALGAE_L4:
        switch (currentRobotState) {
          case PREP_CORAL_WITH_ALGAE_L4:
            return new PrepCoralWithAlgaeL4(subStateMachine);
        }
        break;
      case PREP_CORAL_ZERO_WITH_ALGAE:
        switch (currentRobotState) {
          case PREP_CORAL_ZERO_WITH_ALGAE:
            return new PrepCoralZeroWithAlgae(subStateMachine);
        }
        break;
      case PREP_ALGAE_NET:
        switch (currentRobotState) {
          case PREP_ALGAE_NET:
            return new PrepAlgaeNet(subStateMachine);
        }
        break;
      case PREP_ALGAE_PROCESSOR:
        switch (currentRobotState) {
          case PREP_ALGAE_PROCESSOR:
            return new PrepAlgaeProcessor(subStateMachine);
        }
        break;
      case PREP_ALGAE_ZERO:
        switch (currentRobotState) {
          case PREP_ALGAE_ZERO:
            return new PrepAlgaeZero(subStateMachine);
        }
        break;
      case PREP_ALGAE_NET_WITH_CORAL:
        switch (currentRobotState) {
          case PREP_ALGAE_NET_WITH_CORAL:
            return new PrepAlgaeNetWithCoral(subStateMachine);
        }
        break;
      case PREP_ALGAE_PROCESSOR_WITH_CORAL:
        switch (currentRobotState) {
          case PREP_ALGAE_PROCESSOR_WITH_CORAL:
            return new PrepAlgaeProcessorWithCoral(subStateMachine);
        }
        break;
      case PREP_ALGAE_ZERO_WITH_CORAL:
        switch (currentRobotState) {
          case PREP_ALGAE_ZERO_WITH_CORAL:
            return new PrepAlgaeZeroWithCoral(subStateMachine);
        }
        break;
      case HAS_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL:
            return new HasCoral(subStateMachine);
        }
        break;
      case HAS_ALGAE:
        switch (currentRobotState) {
          case HAS_ALGAE:
            return new HasAlgae(subStateMachine);
        }
        break;
      case HAS_CORAL_AND_ALGAE:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
            return new HasCoralAndAlgae(subStateMachine);
        }
        break;
      case EJECTING:
        switch (currentRobotState) {
          case EJECTING:
            return new Ejecting(subStateMachine);
        }
        break;
      case SCORING_CORAL:
        switch (currentRobotState) {
          case SCORING_CORAL:
            return new ScoringCoral(subStateMachine);
        }
        break;
      case SCORING_ALGAE:
        switch (currentRobotState) {
          case SCORING_ALGAE:
            return new ScoringAlgae(subStateMachine);
        }
        break;
      case SCORING_ALGAE_WITH_CORAL:
        switch (currentRobotState) {
          case SCORING_ALGAE_WITH_CORAL:
            return new ScoringAlgaeWithCoral(subStateMachine);
        }
        break;
      case SCORING_CORAL_WITH_ALGAE:
        switch (currentRobotState) {
          case SCORING_CORAL_WITH_ALGAE:
            return new ScoringCoralWithAlgae(subStateMachine);
        }
        break;
      case CLEAN_HIGH_WITH_CORAL:
        switch (currentRobotState) {
          case CLEAN_HIGH_WITH_CORAL:
            return new CleanHighWithCoral(subStateMachine);
        }
        break;
      case CLEAN_LOW_WITH_CORAL:
        switch (currentRobotState) {
          case CLEAN_LOW_WITH_CORAL:
            return new CleanLowWithCoral(subStateMachine);
        }
        break;
      case CLEAN_HIGH:
        switch (currentRobotState) {
          case CLEAN_HIGH:
            return new CleanHigh(subStateMachine);
        }
        break;
      case CLEAN_LOW:
        switch (currentRobotState) {
          case CLEAN_LOW:
            return new CleanLow(subStateMachine);
        }
        break;
      case INTAKE_CORAL_GROUND:
        switch (currentRobotState) {
          case INTAKE_CORAL_GROUND:
            return new IntakeCoralGround(subStateMachine);
        }
        break;
      case INTAKE_CORAL_STATION:
        switch (currentRobotState) {
          case INTAKE_CORAL_STATION:
            return new IntakeCoralStation(subStateMachine);
        }
        break;
      case INTAKE_ALGAE_GROUND:
        switch (currentRobotState) {
          case INTAKE_ALGAE_GROUND:
            return new IntakeAlgaeGround(subStateMachine);
        }
        break;
      case INTAKE_CORAL_WITH_ALGAE_GROUND:
        switch (currentRobotState) {
          case INTAKE_CORAL_WITH_ALGAE_GROUND:
            return new IntakeCoralWithAlgaeGround(subStateMachine);
        }
        break;
      case INTAKE_ALGAE_WITH_CORAL_GROUND:
        switch (currentRobotState) {
          case INTAKE_ALGAE_WITH_CORAL_GROUND:
            return new IntakeAlgaeWithCoralGround(subStateMachine);
        }
        break;
      case INTAKE_CORAL_WITH_ALGAE_STATION:
        switch (currentRobotState) {
          case INTAKE_CORAL_WITH_ALGAE_STATION:
            return new IntakeCoralWithAlgaeStation(subStateMachine);
        }
        break;
    }
    return Commands
        .print("ITS SO OVER D: Invalid State Provided, Blame Eli. Attempted to go to: " + desiredState.toString()
            + " while at " + currentRobotState.toString());
  }

  public enum DriverState {
    MANUAL
    // TODO: Add other driver states as needed
  }

  public enum RobotState {
    NONE,
    PREP_CLIMB,
    CLIMBINGPREP_CORAL_L1,
    PREP_CORAL_L2,
    PREP_CORAL_L3,
    PREP_CORAL_L4,
    PREP_CORAL_ZERO,
    PREP_CORAL_WITH_ALGAE_L1,
    PREP_CORAL_WITH_ALGAE_L2,
    PREP_CORAL_WITH_ALGAE_L3,
    PREP_CORAL_WITH_ALGAE_L4,
    PREP_CORAL_ZERO_WITH_ALGAE,
    PREP_ALGAE_NET,
    PREP_ALGAE_PROCESSOR,
    PREP_ALGAE_ZERO,
    PREP_ALGAE_NET_WITH_CORAL,
    PREP_ALGAE_PROCESSOR_WITH_CORAL,
    PREP_ALGAE_ZERO_WITH_CORAL,
    HAS_CORAL,
    HAS_ALGAE,
    HAS_CORAL_AND_ALGAE,
    EJECTING,
    SCORING_CORAL,
    SCORING_ALGAE,
    SCORING_ALGAE_WITH_CORAL,
    SCORING_CORAL_WITH_ALGAE,
    CLEAN_HIGH_WITH_CORAL,
    CLEAN_LOW_WITH_CORAL,
    CLEAN_HIGH,
    CLEAN_LOW,
    INTAKE_CORAL_GROUND,
    INTAKE_CORAL_STATION,
    INTAKE_ALGAE_GROUND,
    INTAKE_CORAL_WITH_ALGAE_GROUND,
    INTAKE_ALGAE_WITH_CORAL_GROUND,
    INTAKE_CORAL_WITH_ALGAE_STATION
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
