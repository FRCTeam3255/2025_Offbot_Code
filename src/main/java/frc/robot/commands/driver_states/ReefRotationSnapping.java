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
import frc.robot.subsystems.DriverStateMachine.DriverState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefRotationSnapping extends Command {
  /** Creates a new ReefRotationSnapping. */
  double redAllianceMultiplier = 1;
  boolean isRedAlliance = constField.isRedAlliance();
  Drivetrain subDrivetrain;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  DriverStateMachine subDriverStateMachine;
  boolean isOpenLoop;

  Pose2d getDesiredReefPose() {
    return subDrivetrain.getDesiredPose(constField.getReefPositions(constField.isRedAlliance()).get());
  }

  public ReefRotationSnapping(Drivetrain subDrivetrain, DriverStateMachine subDriverStateMachine,
      DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier rotationAxis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subDrivetrain = subDrivetrain;
    this.subDriverStateMachine = subDriverStateMachine;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    addRequirements(this.subDrivetrain);
    addRequirements(this.subDriverStateMachine);
    isOpenLoop = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    redAllianceMultiplier = constField.isRedAlliance() ? -1 : 1;
    subDriverStateMachine.setDriverState(DriverState.REEF_ROTATION_SNAPPING);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LinearVelocity xVelocity = Units.MetersPerSecond.of(xAxis.getAsDouble());
    LinearVelocity yVelocity = Units.MetersPerSecond.of(-yAxis.getAsDouble());

    Pose2d closestPose = subDrivetrain.getDesiredPose(constField.getAlgaePositions(isRedAlliance).get());
    subDrivetrain.rotationalAlign(
        isRedAlliance,
        closestPose,
        xVelocity,
        yVelocity,
        isOpenLoop);
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
