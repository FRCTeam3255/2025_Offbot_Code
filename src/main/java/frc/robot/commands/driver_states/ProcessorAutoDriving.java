package frc.robot.commands.driver_states;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DriverStateMachine.DriverState;
import frc.robot.Field;

public class ProcessorAutoDriving extends Command {
  double redAllianceMultiplier = 1;
  Drivetrain subDrivetrain;
  DriverStateMachine subDriverStateMachine;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  boolean isOpenLoop;
  boolean isRedAlliance = Field.isRedAlliance();
  Motion globalMotion;

  public ProcessorAutoDriving(Drivetrain subDrivetrain, DriverStateMachine subDriverStateMachine,
      DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier rotationAxis, Motion subMotion) {
    this.subDrivetrain = subDrivetrain;
    this.subDriverStateMachine = subDriverStateMachine;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    globalMotion = subMotion;
    addRequirements(this.subDrivetrain, this.subDriverStateMachine);
    isOpenLoop = true;
  }

  @Override
  public void initialize() {
    redAllianceMultiplier = Field.isRedAlliance() ? -1 : 1;
    subDriverStateMachine.setDriverState(DriverState.PROCESSOR_AUTO_DRIVING);
  }

  @Override
  public void execute() {
    boolean isInAutoDriveZone = subDrivetrain.isInAutoDriveZone(
        Field.PROCESSOR_AUTO_DRIVE_MAX_DISTANCE,
        Field.getProcessorPose().get());
    LinearVelocity xVelocity = Units.MetersPerSecond
        .of(xAxis.getAsDouble() * constDrivetrain.REAL_DRIVE_SPEED.in(Units.MetersPerSecond) * redAllianceMultiplier
            - globalMotion.getLiftPosition().in(Units.Meters) / constMotion.HEIGHT_DIVIDER.in(Units.Meters));
    LinearVelocity yVelocity = Units.MetersPerSecond
        .of(-yAxis.getAsDouble()
            * constDrivetrain.REAL_DRIVE_SPEED.in(Units.MetersPerSecond) * redAllianceMultiplier
            - globalMotion.getLiftPosition().in(Units.Meters) / constMotion.HEIGHT_DIVIDER.in(Units.Meters));
    if (isInAutoDriveZone) {
      Pose2d closestPose = subDrivetrain
          .getDesiredPose(Field.getProcessorPose().get());
      subDrivetrain.autoAlign(Field.isRedAlliance(),
          closestPose,
          xVelocity,
          yVelocity,
          isOpenLoop,
          true,
          false);
      subDriverStateMachine.setDriverState(DriverState.PROCESSOR_AUTO_DRIVING);
    } else {
      subDrivetrain.rotationalAlign(isRedAlliance,
          subDrivetrain.getDesiredPose(Field.getProcessorPose().get()),
          xVelocity,
          yVelocity,
          isOpenLoop);
      subDriverStateMachine.setDriverState(DriverState.PROCESSOR_ROTATION_SNAPPING);
    }
  }

  @Override
  public void end(boolean interrupted) {
    subDrivetrain.neutralDriveOutputs();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
