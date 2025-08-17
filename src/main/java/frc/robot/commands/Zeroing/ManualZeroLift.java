// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Zeroing;

import frc.robot.subsystems.Motion;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constMotion;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualZeroLift extends Command {
  Motion globalMotion;

  Time zeroingTimestamp = Units.Seconds.of(0);
  boolean zeroingSuccess = false;
  AngularVelocity lastLiftVelocity = Units.RotationsPerSecond.of(0);

  /** Creates a new ManualZeroLift. */
  public ManualZeroLift(Motion subMotion) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalMotion = subMotion;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    globalMotion.hasLiftZeroed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    globalMotion.setLiftCoastMode(true);

    // Check if we have raised the elevator above a certain speed
    if (globalMotion.getLiftVelocity().gte(constMotion.MANUAL_ZEROING_START_VELOCITY)
        || globalMotion.attemptingZeroing) {
      // Enter zeroing mode!
      if (!globalMotion.attemptingZeroing) {
        globalMotion.attemptingZeroing = true;
        zeroingTimestamp = Units.Seconds.of(Timer.getFPGATimestamp());
        System.out.println("Elevator Zeroing Started!");
      }

      // Check if time elapsed is too high (zeroing timeout)
      if (Units.Seconds.of(Timer.getFPGATimestamp()).minus(zeroingTimestamp).gte(constMotion.ZEROING_TIMEOUT)) {
        globalMotion.attemptingZeroing = false;
        System.out.println("Elevator Zeroing Failed :(");
      } else {
        boolean deltaLiftVelocity = globalMotion.getLiftVelocity().minus(lastLiftVelocity)
            .lte(constMotion.MANUAL_ZEROING_DELTA_VELOCITY);

        if (deltaLiftVelocity && lastLiftVelocity.lte(Units.RotationsPerSecond.of(0))) {
          zeroingSuccess = true;
        } else {
          lastLiftVelocity = globalMotion.getLiftVelocity();
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted && zeroingSuccess) {
      globalMotion.hasLiftZeroed = true;
      globalMotion.resetLiftSensorPosition(constMotion.LIFT_ZEROED_POSITION);
      globalMotion.setLiftCoastMode(false);
      System.out.println("Elevator Zeroing Successful!!!! Yippee and hooray!!! :3");
    } else {
      System.out.println("Elevator was never zeroed :((( blame eli");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return zeroingSuccess && globalMotion.isLiftVelocityZero();
  }
}
