// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Zeroing;

import frc.robot.Constants.constMotion;
import frc.robot.subsystems.Motion;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualZeroWrist extends Command {
  /** Creates a new ManualZeroWrist. */
  Motion globalMotion;

  boolean zeroingSuccess = false;
  Time zeroingTimestamp = Units.Seconds.of(0);
  AngularVelocity lastWristVelocity = Units.RotationsPerSecond.of(0);

  public ManualZeroWrist(Motion subMotion) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalMotion = subMotion;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    zeroingSuccess = false;
    globalMotion.hasWristZeroed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    globalMotion.setWristCoastMode(true);
    // Check if we have raised the Wrist above a certain speed
    if (globalMotion.getWristVelocity().gte(constMotion.MANUAL_ZEROING_START_VELOCITY)
        || globalMotion.attemptingZeroing) {
      // Enter zeroing mode!
      if (!globalMotion.attemptingZeroing) {
        globalMotion.attemptingZeroing = true;
        zeroingTimestamp = Units.Seconds.of(Timer.getFPGATimestamp());
        System.out.println("Wrist Zeroing Started!");
      }

      // Check if time elapsed is too high (zeroing timeout)
      if (Units.Seconds.of(Timer.getFPGATimestamp()).minus(zeroingTimestamp).gte(constMotion.ZEROING_TIMEOUT)) {
        globalMotion.attemptingZeroing = false;
        System.out.println("Wrist Zeroing Failed :(");
      } else {
        boolean deltaWristVelocity = globalMotion.getWristVelocity().minus(lastWristVelocity)
            .lte(constMotion.MANUAL_ZEROING_DELTA_VELOCITY);

        if (deltaWristVelocity && lastWristVelocity.lte(Units.RotationsPerSecond.of(0))) {
          zeroingSuccess = true;
        } else {
          lastWristVelocity = globalMotion.getWristVelocity();
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted && zeroingSuccess) {
      globalMotion.hasWristZeroed = true;
      globalMotion.resetWristSensorPosition(constMotion.WRIST_ZEROED_POSITION);
      globalMotion.setWristCoastMode(false);
      System.out.println("Wrist Zeroing Successful!!!! Yippee and hooray!!! :3");
    } else {
      System.out.println("Wrist was never zeroed :((( blame eli");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return zeroingSuccess && globalMotion.isWristVelocityZero();
  }
}
