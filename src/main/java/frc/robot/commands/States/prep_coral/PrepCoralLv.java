// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.prep_coral;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismPositionGroup;
import frc.robot.Constants.constMechanismPositions;
import frc.robot.Constants.constRotorsSpeeds;
import frc.robot.Field;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Motion;
import frc.robot.subsystems.Rotors;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepCoralLv extends Command {
  /** Creates a new PrepCoralLv. */
  Drivetrain globalDrivetrain;
  Pose2d closestPoseByRotation;
  MechanismPositionGroup prepL2;
  MechanismPositionGroup prepL3;
  MechanismPositionGroup prepL4;
  int targetLevel;

  public PrepCoralLv(int level) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalDrivetrain = Drivetrain.getInstance();
    addRequirements(StateMachine.getInstance());
    targetLevel = level;
  }

  @Override
  public void initialize() {
    if (globalDrivetrain.isActionBackwards(
        Field.FieldElementGroups.REEF_POSES.getAll()) == true) {
      prepL2 = constMechanismPositions.PREP_CORAL_L2_BACKWARDS;
      prepL3 = constMechanismPositions.PREP_CORAL_L3_BACKWARDS;
      prepL4 = constMechanismPositions.PREP_CORAL_L4_BACKWARDS;
    } else {
      prepL2 = constMechanismPositions.PREP_CORAL_L2_FORWARDS;
      prepL3 = constMechanismPositions.PREP_CORAL_L3_FORWARDS;
      prepL4 = constMechanismPositions.PREP_CORAL_L4_FORWARDS;

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (targetLevel == 0) {
      Motion.getInstance().setAllPosition(constMechanismPositions.PREP_CORAL_ZERO);
      StateMachine.getInstance().setRobotState(RobotState.PREP_CORAL_ZERO);
    } else if (targetLevel == 1) {
      StateMachine.getInstance().setRobotState(RobotState.PREP_CORAL_L1);
      Rotors.getInstance().setAlgaeIntakeMotorSpeed(constRotorsSpeeds.L1_CORAL_HOLD_SPEED);
      Rotors.getInstance().setCoralIntakeMotorSpeed(constRotorsSpeeds.CORAL_L1_CORAL_HOLD_SPEED);
      Motion.getInstance().setAllPosition(constMechanismPositions.PREP_CORAL_L1);
    } else if (targetLevel == 2) {
      Motion.getInstance().setAllPosition(prepL2);
      StateMachine.getInstance().setRobotState(RobotState.PREP_CORAL_L2);
    } else if (targetLevel == 3) {
      Motion.getInstance().setAllPosition(prepL3);
      StateMachine.getInstance().setRobotState(RobotState.PREP_CORAL_L3);
    } else if (targetLevel == 4) {
      Motion.getInstance().setAllPosition(prepL4);
      StateMachine.getInstance().setRobotState(RobotState.PREP_CORAL_L4);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
