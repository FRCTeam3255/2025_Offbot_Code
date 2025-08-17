// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driver_states;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.DriverStateMachine;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefAutoDriving extends Command {
  /** Creates a new ReefAutoDriving. */
  double redAllianceMultiplier = 1;
  boolean isRedAlliance = constField.isRedAlliance();
  Drivetrain subDrivetrain;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  DriverStateMachine subDriverStateMachine;
  boolean isOpenLoop;
  boolean leftBranch;

  public ReefAutoDriving(Drivetrain subDrivetrain, DriverStateMachine subDriverStateMachine,
      DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier rotationAxis, boolean leftBranch) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subDrivetrain = subDrivetrain;
    this.subDriverStateMachine = subDriverStateMachine;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    addRequirements(this.subDrivetrain);
    addRequirements(this.subDriverStateMachine);
    isOpenLoop = true;
    this.leftBranch = leftBranch;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    redAllianceMultiplier = constField.isRedAlliance() ? -1 : 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LinearVelocity xVelocity = Units.MetersPerSecond.of(xAxis.getAsDouble() * redAllianceMultiplier);
    LinearVelocity yVelocity = Units.MetersPerSecond.of(-yAxis.getAsDouble() * redAllianceMultiplier);
    Pose2d closestPose;

    if (leftBranch == true) {
      closestPose = subDrivetrain.getDesiredPose(constField.getLeftReefPositions(isRedAlliance).get());
      subDriverStateMachine.setDriverState(DriverStateMachine.DriverState.REEF_AUTO_DRIVING_LEFT);
    } else {
      closestPose = subDrivetrain.getDesiredPose(constField.getRightReefPositions(isRedAlliance).get());
      subDriverStateMachine.setDriverState(DriverStateMachine.DriverState.REEF_AUTO_DRIVING_RIGHT);
    }

    subDrivetrain.autoAlign(isRedAlliance,
        closestPose,
        xVelocity,
        yVelocity,
        isOpenLoop,
        false,
        false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subDrivetrain.neutralDriveOutputs();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
