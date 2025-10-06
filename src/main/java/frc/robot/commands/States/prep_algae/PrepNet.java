// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States.prep_algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismPositionGroup;
import frc.robot.Constants.constMechanismPositions;
import frc.robot.Field;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Motion;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrepNet extends Command {
  MechanismPositionGroup prepNet;

  public PrepNet() {
    addRequirements(StateMachine.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Drivetrain.getInstance().isActionBackwards(Field.FieldElementGroups.NET_POSES.getAll())) {
      prepNet = constMechanismPositions.PREP_ALGAE_NET_BACKWARDS;
    } else {
      prepNet = constMechanismPositions.PREP_ALGAE_NET_FORWARDS;
    }
    StateMachine.getInstance().setRobotState(RobotState.PREP_ALGAE_NET);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Motion.getInstance().setAllPosition(prepNet);
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
