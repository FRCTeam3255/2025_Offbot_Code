// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Zeroing;

import frc.robot.subsystems.Motion;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constMotion;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroPivot extends Command {
  Motion globalMotion;

  Time zeroingTimestamp;
  boolean hasPivotZeroed = false;

  /** Creates a new ZeroPivot. */
  public ZeroPivot(Motion subMotion) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalMotion = subMotion;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalMotion.setPivotVoltage(Units.Volts.zero());
    zeroingTimestamp = Units.Seconds.zero();
    hasPivotZeroed = globalMotion.hasPivotZeroed;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    globalMotion.setPivotVoltage(constMotion.ZEROING_VOLTAGE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    globalMotion.setPivotVoltage(Units.Volts.zero());

    if (!interrupted) {
      globalMotion.resetPivotSensorPosition(constMotion.PIVOT_ZEROED_POSITION);
      globalMotion.hasPivotZeroed = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (hasPivotZeroed) {
      return true;
    }

    // If the current velocity is low enough to be considered as zeroed
    if (globalMotion.getPivotVelocity().lt(constMotion.ZEROED_VELOCITY)) {
      // And this is the first loop it has happened, begin the timer
      if (zeroingTimestamp.equals(Units.Seconds.zero())) {
        zeroingTimestamp = Units.Seconds.of(Timer.getFPGATimestamp());
        return false;
      }

      // If this isn't the first loop, return if it has been below the threshold for
      // long enough
      return (Units.Seconds.of(Timer.getFPGATimestamp()).minus(zeroingTimestamp).gte(constMotion.ZEROED_TIME));
    }

    // If the above wasn't true, we have gained too much velocity, so we aren't at 0
    // & need to restart the timer
    zeroingTimestamp = Units.Seconds.zero();
    return false;
  }
}
