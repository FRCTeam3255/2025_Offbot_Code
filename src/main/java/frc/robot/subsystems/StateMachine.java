// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.commands.States.*;
import frc.robot.commands.States.climbing.*;
import frc.robot.commands.States.first_scoring_element.*;
import frc.robot.commands.States.hold_scoring_elements.*;
import frc.robot.commands.States.prep_algae.*;
import frc.robot.commands.States.prep_coral.*;
import frc.robot.commands.States.scoring.*;
import frc.robot.commands.States.second_scoring_element.*;

@Logged
public class StateMachine extends SubsystemBase {
  public static DriverState currentDriverState;
  public static RobotState currentRobotState;
  @NotLogged
  Drivetrain subDrivetrain;
  @NotLogged
  Motion subMotion;
  @NotLogged
  Rotors subRotors;
  @NotLogged
  StateMachine subStateMachine = this;

  /** Creates a new StateMachine. */
  public StateMachine(Drivetrain subDrivetrain, Rotors subIntake,
      Motion subElevator) {
    currentRobotState = RobotState.NONE;
    currentDriverState = DriverState.MANUAL;
    this.subRotors = subIntake;
    this.subMotion = subElevator;
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
          case PREP_CLIMB:
          case SCORING_CORAL:
          case SCORING_ALGAE:
          case INTAKE_CORAL_GROUND:
          case INTAKE_ALGAE_GROUND:
          case INTAKE_CORAL_STATION:
          case CLEAN_HIGH:
          case CLEAN_LOW:
          case EJECTING:
            return new None(subStateMachine, subMotion, subRotors);
        }

        break;

      // climbing states

      case PREP_CLIMB:
        switch (currentRobotState) {
          case NONE:
          case HAS_CORAL:
          case HAS_ALGAE:
          case HAS_CORAL_AND_ALGAE:
            return new PrepClimb(subStateMachine, subMotion, subRotors);

        }
        break;

      case CLIMBING:
        switch (currentRobotState) {
          case PREP_CLIMB:
            return new Climbing(subStateMachine, subMotion);

        }
        break;

      // Prep Coral only
      case PREP_CORAL_ZERO:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
            return new PrepCoralZero(subStateMachine, subMotion, subRotors);

        }
        break;

      case PREP_CORAL_L1:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
          case PREP_CORAL_ZERO:
            return new PrepCoralLv(subStateMachine, subMotion, subRotors,
                constMechanismPositions.ELEVATOR_CORAL_L1_HEIGHT);

        }
        break;

      case PREP_CORAL_L2:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L1:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
          case PREP_CORAL_ZERO:
            return new PrepCoralLv(subStateMachine, subMotion, subRotors,
                constMechanismPositions.ELEVATOR_CORAL_L2_HEIGHT);

        }
        break;

      case PREP_CORAL_L3:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L4:
          case PREP_CORAL_ZERO:
            return new PrepCoralLv(subStateMachine, subMotion, subRotors,
                constMechanismPositions.ELEVATOR_CORAL_L3_HEIGHT);

        }
        break;

      case PREP_CORAL_L4:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_ZERO:
            return new PrepCoralLv(subStateMachine, subMotion, subRotors,
                constMechanismPositions.ELEVATOR_CORAL_L4_HEIGHT);

        }
        break;

      // prep Coral with Algae

      case PREP_CORAL_L1_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_ALGAE_PROCESSOR_WITH_CORAL:
          case PREP_ALGAE_NET_WITH_CORAL:
            return new PrepCoralWithAlgae(subStateMachine, subMotion, subRotors,
                constMechanismPositions.ELEVATOR_CORAL_L1_HEIGHT);

        }
        break;

      case PREP_CORAL_L2_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_L1_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_ALGAE_PROCESSOR_WITH_CORAL:
          case PREP_ALGAE_NET_WITH_CORAL:
            return new PrepCoralWithAlgae(subStateMachine, subMotion, subRotors,
                constMechanismPositions.ELEVATOR_CORAL_L2_HEIGHT);

        }
        break;

      case PREP_CORAL_L3_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_L1_WITH_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_ALGAE_PROCESSOR_WITH_CORAL:
          case PREP_ALGAE_NET_WITH_CORAL:
            return new PrepCoralWithAlgae(subStateMachine, subMotion, subRotors,
                constMechanismPositions.ELEVATOR_CORAL_L3_HEIGHT);

        }
        break;

      case PREP_CORAL_L4_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_L1_WITH_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
          case PREP_ALGAE_PROCESSOR_WITH_CORAL:
          case PREP_ALGAE_NET_WITH_CORAL:
            return new PrepCoralWithAlgae(subStateMachine, subMotion, subRotors,
                constMechanismPositions.ELEVATOR_CORAL_L4_HEIGHT);

        }
        break;

      case PREP_CORAL_ZERO_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_CORAL_L1_WITH_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_ALGAE_PROCESSOR_WITH_CORAL:
          case PREP_ALGAE_NET_WITH_CORAL:
            return new PrepCoralZeroWithAlgae(subStateMachine, subMotion, subRotors);

        }
        break;

      // prep Algae only

      case PREP_ALGAE_NET:
        switch (currentRobotState) {
          case HAS_ALGAE:
          case PREP_ALGAE_PROCESSOR:
          case PREP_ALGAE_ZERO:
            return new PrepNet(subStateMachine, subMotion, subRotors);

        }
        break;

      case PREP_ALGAE_PROCESSOR:
        switch (currentRobotState) {
          case HAS_ALGAE:
          case PREP_ALGAE_ZERO:
          case PREP_ALGAE_NET:
            return new PrepProcessor(subStateMachine, subMotion, subRotors);

        }
        break;

      case PREP_ALGAE_ZERO:
        switch (currentRobotState) {
          case HAS_ALGAE:
          case PREP_ALGAE_NET:
          case PREP_ALGAE_PROCESSOR:
            return new PrepAlgaeZero(subStateMachine, subMotion, subRotors);

        }
        break;

      // prep Algae with Coral

      case PREP_ALGAE_NET_WITH_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_ALGAE_PROCESSOR_WITH_CORAL:
          case PREP_CORAL_L1_WITH_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
            return new PrepNetWithCoral(subStateMachine, subMotion, subRotors);

        }
        break;

      case PREP_ALGAE_PROCESSOR_WITH_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL_AND_ALGAE:
          case PREP_ALGAE_NET_WITH_CORAL:
          case PREP_CORAL_L1_WITH_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:
            return new PrepProcessorWithCoral(subStateMachine, subMotion, subRotors);

        }
        break;

      // holding 1 game piece

      case HAS_CORAL:
        switch (currentRobotState) {
          case INTAKE_CORAL_GROUND:
          case INTAKE_CORAL_STATION:
          case SCORING_ALGAE_WITH_CORAL:
          case INTAKE_ALGAE_GROUND_WITH_CORAL:
            return new HasCoral(subStateMachine);
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
            return new HasAlgae(subStateMachine);
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
            return new HasCoralAndAlgae(subStateMachine);
        }
        break;

      // manipulating 1 game piece

      case SCORING_CORAL:
        switch (currentRobotState) {
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
          case PREP_CORAL_ZERO:

            return new ScoringCoral(subStateMachine, subRotors);
        }
        break;

      case SCORING_ALGAE:
        switch (currentRobotState) {
          case PREP_ALGAE_NET:
          case PREP_ALGAE_PROCESSOR:
          case PREP_ALGAE_ZERO:
            return new ScoringAlgae(subStateMachine, subRotors);
        }
        break;

      case CLEAN_HIGH:
        switch (currentRobotState) {
          case CLEAN_LOW:
          case INTAKE_ALGAE_GROUND:
          case NONE:
            return new CleanHigh(subStateMachine, subMotion, subRotors);
        }
        break;

      case CLEAN_LOW:
        switch (currentRobotState) {
          case NONE:
          case INTAKE_ALGAE_GROUND:
          case CLEAN_HIGH:
            return new CleanLow(subStateMachine, subMotion, subRotors);
        }
        break;

      case INTAKE_CORAL_STATION:
        switch (currentRobotState) {
          case NONE:
          case INTAKE_CORAL_GROUND:
            return new IntakeCoralStation(subStateMachine, subMotion, subRotors);
        }
        break;

      case INTAKE_ALGAE_GROUND:
        switch (currentRobotState) {
          case NONE:
          case CLEAN_HIGH:
          case CLEAN_LOW:
            return new IntakeAlgaeGround(subStateMachine, subMotion, subRotors);
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
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
          case PREP_CORAL_L1_WITH_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_CORAL_ZERO:
            return new Ejecting(subStateMachine, subRotors);
        }
        break;

      case SCORING_ALGAE_WITH_CORAL:
        switch (currentRobotState) {
          case PREP_ALGAE_NET_WITH_CORAL:
          case PREP_ALGAE_PROCESSOR_WITH_CORAL:
          case PREP_CORAL_ZERO_WITH_ALGAE:
            return new ScoringAlgaeWithCoral(subStateMachine, subRotors);
        }
        break;

      case SCORING_CORAL_WITH_ALGAE:
        switch (currentRobotState) {
          case PREP_CORAL_L1_WITH_ALGAE:
          case PREP_CORAL_L2_WITH_ALGAE:
          case PREP_CORAL_L3_WITH_ALGAE:
          case PREP_CORAL_L4_WITH_ALGAE:
          case PREP_CORAL_ZERO_WITH_ALGAE:

            return new ScoringCoralWithAlgae(subStateMachine, subRotors);
        }
        break;

      case CLEAN_HIGH_WITH_CORAL:
        switch (currentRobotState) {
          case CLEAN_LOW_WITH_CORAL:
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
          case HAS_CORAL:
          case INTAKE_ALGAE_GROUND_WITH_CORAL:
            return new CleanHighWithCoral(subStateMachine, subMotion, subRotors);
        }
        break;

      case CLEAN_LOW_WITH_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL:
          case PREP_CORAL_L1:
          case PREP_CORAL_L2:
          case PREP_CORAL_L3:
          case PREP_CORAL_L4:
          case CLEAN_HIGH_WITH_CORAL:
          case INTAKE_ALGAE_GROUND_WITH_CORAL:
            return new CleanLowWithCoral(subStateMachine, subMotion, subRotors);
        }
        break;

      case INTAKE_CORAL_GROUND:
        switch (currentRobotState) {
          case NONE:
          case INTAKE_CORAL_STATION:
            return new IntakeCoralGround(subStateMachine, subMotion, subRotors);
        }
        break;

      case INTAKE_CORAL_GROUND_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_ALGAE:
          case INTAKE_CORAL_STATION_WITH_ALGAE:
            return new IntakeCoralGroundWithAlgae(subStateMachine, subMotion, subRotors);
        }
        break;

      case INTAKE_ALGAE_GROUND_WITH_CORAL:
        switch (currentRobotState) {
          case HAS_CORAL:
          case CLEAN_HIGH_WITH_CORAL:
          case CLEAN_LOW_WITH_CORAL:
          case PREP_CORAL_ZERO:
            return new IntakeAlgaeGroundWithCoral(subStateMachine, subMotion, subRotors);
        }
        break;
      case INTAKE_CORAL_STATION_WITH_ALGAE:
        switch (currentRobotState) {
          case HAS_ALGAE:
          case INTAKE_CORAL_GROUND_WITH_ALGAE:
            return new IntakeCoralStationWithAlgae(subStateMachine, subMotion, subRotors);
        }
        break;
    }
    return Commands.print("ITS SO OVER D: Invalid State Provided, Blame Eli. Attempted to go to: "
        + desiredState.toString() + " while at " + currentRobotState.toString());
  }

  public enum DriverState {
    MANUAL,
    REEF_ROTATION_SNAPPING,
    CORAL_STATION_ROTATION_SNAPPING,
    REEF_AUTO_DRIVING,
    CORAL_STATION_AUTO_DRIVING,
    PROCESSOR_ROTATION_SNAPPING,
    PROCESSOR_AUTO_DRIVING,
    NET_ROTATION_SNAPPING,
    NET_AUTO_DRIVING,
    ALGAE_ROTATION_SNAPPING,
    ALGAE_AUTO_DRIVING,
    CAGE_ROTATION_SNAPPING
    // TODO: Add other driver states as needed
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
    PREP_CORAL_L1_WITH_ALGAE,
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
    CLEAN_HIGH,
    CLEAN_LOW,
    INTAKE_CORAL_STATION,
    INTAKE_ALGAE_GROUND,
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
