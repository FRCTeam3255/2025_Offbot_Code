// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Zeroing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constLED;
import frc.robot.Constants.constMotion;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StartingConfig extends Command {
  /** Creates a new StartingConfig. */
  Motion subMotion;
  LED globalLED;
  boolean isAtStartingConfig = false;

  public StartingConfig(Motion subMotion, LED subLED) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subMotion = subMotion;
    globalLED = subLED;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (subMotion.hasPivotZeroed
        && subMotion.getPivotAngle().gte(constMotion.PIVOT_STARTING_CONFIG_VALUE)) {
      subMotion.setPivotCoastMode(false);
      isAtStartingConfig = true;
      subMotion.hasSetStartingConfig = true;
      System.out.println("Elevator Pivot is at starting config! :P" + subMotion.getPivotAngle());
    } else {
      System.out.println("Elevator Pivot is not at starting config!!1!!! :((( blame eli" + subMotion.getPivotAngle());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    globalLED.setLEDMatrix(constLED.IS_AT_STARTING_CONFIG, 3, 1);
    globalLED.setLEDMatrix(constLED.IS_AT_STARTING_CONFIG, 4, 1);
    subMotion.setPivotCoastMode(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isAtStartingConfig;
  }
}
