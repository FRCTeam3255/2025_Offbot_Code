// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Zeroing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constLED;
import frc.robot.Constants.constMotion;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Motion;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StartingConfig extends Command {
  /** Creates a new StartingConfig. */
  boolean isAtStartingConfig = false;

  public StartingConfig() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Motion.getInstance(), LED.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LED.getInstance().setLEDMatrix(constLED.IS_NOT_AT_STARTING_CONFIG, 2, 1);
    LED.getInstance().setLEDMatrix(constLED.IS_NOT_AT_STARTING_CONFIG, 5, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Motion.getInstance().hasPivotZeroed
        && Motion.getInstance().getPivotAngle().gte(constMotion.PIVOT_STARTING_CONFIG_VALUE)) {
      Motion.getInstance().setPivotCoastMode(false);
      isAtStartingConfig = true;
      Motion.getInstance().hasSetStartingConfig = true;
      System.out.println("Elevator Pivot is at starting config! :P" + Motion.getInstance().getPivotAngle());
    } else {
      System.out.println(
          "Elevator Pivot is not at starting config!!1!!! :((( blame eli" + Motion.getInstance().getPivotAngle());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LED.getInstance().setLEDMatrix(constLED.IS_AT_STARTING_CONFIG, 2, 1);
    LED.getInstance().setLEDMatrix(constLED.IS_AT_STARTING_CONFIG, 5, 1);
    Motion.getInstance().setPivotCoastMode(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isAtStartingConfig;
  }
}
