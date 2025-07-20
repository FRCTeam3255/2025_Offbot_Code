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
import frc.robot.commands.states.*;
import frc.robot.subsystems.*;

@Logged
public class StateMachine extends SubsystemBase {
  public static DriverState currentDriverState;
  public static RobotState currentRobotState;
  @NotLogged
  Drivetrain subDrivetrain;
  @NotLogged
  Elevator subElevator;
  @NotLogged
  Intake subIntake;
  @NotLogged
  Climber subClimber;
  @NotLogged
  StateMachine subStateMachine = this;

  /** Creates a new StateMachine. */
  public StateMachine(Drivetrain subDrivetrain, Elevator subElevator, Intake subIntake,
      Climber subClimber) {
    currentRobotState = RobotState.NONE;
    currentDriverState = DriverState.MANUAL;
    this.subElevator = subElevator;
    this.subIntake = subIntake;
    this.subClimber = subClimber;

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

// climbing states

      case PREP_CLIMB:
        switch (currentRobotState) {
          
            return new PrepClimb(subStateMachine);
          
        }
        break;

      case CLIMBING:
        switch (currentRobotState) {
          
            return new Climbing(subStateMachine);
          
        }
        break;

// Prep Coral only        
      case PREP_CORAL_ZERO:   
        switch (currentRobotState) {
  
    return new PrepCoralZero(subStateMachine);
  
}
break;

case PREP_CORAL_L1:   
        switch (currentRobotState) {
  
    return new PrepCoralLv(subStateMachine);
  
}
break;

case PREP_CORAL_L2:   
        switch (currentRobotState) {
  
    return new PrepCoralLv(subStateMachine);
  
}
break;

case PREP_CORAL_L3:   
        switch (currentRobotState) {
  
    return new PrepCoralLv(subStateMachine);
  
}
break;

case PREP_CORAL_L4:   
        switch (currentRobotState) {
  
    return new PrepCoralLv(subStateMachine);
  
}
break;

// prep Coral with Algae
      
case PREP_CORAL_WITH_ALGAE_L1:   
        switch (currentRobotState) {
  
    return new PrepCoralWithAlgae(subStateMachine);
  
}
break;

case PREP_CORAL_WITH_ALGAE_L2:   
        switch (currentRobotState) {
  
    return new PrepCoralWithAlgae(subStateMachine);
  
}
break;

case PREP_CORAL_WITH_ALGAE_L3:   
        switch (currentRobotState) {
  
    return new PrepCoralWithAlgae(subStateMachine);
  
}
break;

case PREP_CORAL_WITH_ALGAE_L4:   
        switch (currentRobotState) {
  
    return new PrepCoralWithAlgae(subStateMachine);
  
}
break;

case PREP_CORAL_ZERO_WITH_ALGAE:   
        switch (currentRobotState) {
  
    return new PrepCoralWithAlgae(subStateMachine);
  
}
break;

// prep Algae only  

case PREP_ALGAE_NET:   
        switch (currentRobotState) {
  
    return new PrepNet(subStateMachine);
  
}
break;

case PREP_ALGAE_PROCESSOR:   
        switch (currentRobotState) {
  
    return new PrepProcessor(subStateMachine);
  
}
break;

case PREP_ALGAE_ZERO:   
        switch (currentRobotState) {
  
    return new PrepZero(subStateMachine);
  
}
break;

// prep Algae with Coral

case PREP_ALGAE_NET_WITH_CORAL:   
        switch (currentRobotState) {
  
    return new PrepNetWithCoral(subStateMachine);
  
}
break;

case PREP_ALGAE_PROCESSOR_WITH_CORAL:   
        switch (currentRobotState) {
  
    return new PrepProcessorWithCoral(subStateMachine);
  
}
break;

case PREP_ALGAE_ZERO_WITH_CORAL:   
        switch (currentRobotState) {
  
    return new PrepZeroWithCoral(subStateMachine);
  
}
break;

// holding 1 game piece

      case HAS_CORAL:
        switch (currentRobotState) {
          
            return new HasCoral(subStateMachine);
        }
        break;

      case HAS_ALGAE:
        switch (currentRobotState) {
         
            return new HasAlgae(subStateMachine);
        }
        break;

// holding 2 game pieces
      case HAS_CORAL_AND_ALGAE:
        switch (currentRobotState) {
          
            return new HasCoralAndAlgae(subStateMachine);
        }
        break;

// manipulating 1 game piece


      case SCORING_CORAL:
        switch (currentRobotState) {
          
            return new ScoringCoral(subStateMachine);
        }
        break;

      case SCORING_ALGAE:
        switch (currentRobotState) {
          
            return new ScoringAlgae(subStateMachine);
        }
        break;

      case CLEAN_HIGH:
        switch (currentRobotState) {
          
            return new CleanHigh(subStateMachine);
        }
        break;

      case CLEAN_LOW:
        switch (currentRobotState) {
          
            return new CleanLow(subStateMachine);
        }
        break;

      case INTAKE_CORAL_STATION:
        switch (currentRobotState) {
          
            return new IntakeCoralStation(subStateMachine);
        }
        break;

      case INTAKE_ALGAE_GROUND:
        switch (currentRobotState) {
          
            return new IntakeAlgaeGround(subStateMachine);
        }
        break;

// manipulating 2 game pieces
      case EJECTING:
        switch (currentRobotState) {
          
            return new Ejecting(subStateMachine);
        }
        break;

      case SCORING_ALGAE_WITH_CORAL:
        switch (currentRobotState) {
          
            return new ScoringAlgaeWithCoral(subStateMachine);
        }
        break;

      case SCORING_CORAL_WITH_ALGAE:
        switch (currentRobotState) {
          
            return new ScoringCoralWithAlgae(subStateMachine);
        }
        break;

      case CLEAN_HIGH_WITH_CORAL:
        switch (currentRobotState) {
          
            return new CleanHighWithCoral(subStateMachine);
        }
        break;

      case CLEAN_LOW_WITH_CORAL:
        switch (currentRobotState) {
          
            return new CleanLowWithCoral(subStateMachine);
        }
        break;

      case INTAKE_CORAL_GROUND:
        switch (currentRobotState) {
          
            return new IntakeCoralGround(subStateMachine);
        }
        break;

      case INTAKE_CORAL_WITH_ALGAE_GROUND:
        switch (currentRobotState) {
          
            return new IntakeCoralWithAlgaeGround(subStateMachine);
        }
        break;

      case INTAKE_ALGAE_WITH_CORAL_GROUND:
        switch (currentRobotState) {
          
            return new IntakeAlgaeWithCoralGround(subStateMachine);
        }
        break;

    }return Commands.print("ITS SO OVER D: Invalid State Provided, Blame Eli. Attempted to go to: "+desiredState.toString()+" while at "+currentRobotState.toString());
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
    PREP_CORAL_WITH_ALGAE_L1,
    PREP_CORAL_WITH_ALGAE_L2,
    PREP_CORAL_WITH_ALGAE_L3,
    PREP_CORAL_WITH_ALGAE_L4,
    PREP_CORAL_ZERO_WITH_ALGAE,
    // prep Algae only
    PREP_ALGAE_NET,
    PREP_ALGAE_PROCESSOR,
    PREP_ALGAE_ZERO,
    // prep Algae with Coral
    PREP_ALGAE_NET_WITH_CORAL,
    PREP_ALGAE_PROCESSOR_WITH_CORAL,
    PREP_ALGAE_ZERO_WITH_CORAL,
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
    INTAKE_CORAL_WITH_ALGAE_GROUND,
    INTAKE_ALGAE_WITH_CORAL_GROUND,

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
