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

public class NetAutoDriving extends Command {
  Drivetrain subDrivetrain;
  DriverStateMachine subDriverStateMachine;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  boolean isOpenLoop;
  double redAllianceMultiplier = 1;
  Motion globalMotion;

  public NetAutoDriving(Drivetrain subDrivetrain, DriverStateMachine subDriverStateMachine,
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
    subDriverStateMachine.setDriverState(DriverState.NET_AUTO_DRIVING);
  }

  @Override
  public void execute() {
    boolean isInAutoDriveZone = subDrivetrain.isInAutoDriveZone(
        Field.NET_AUTO_DRIVE_MAX_DISTANCE,
        Field.getNetPositions().get());
    LinearVelocity xVelocity = Units.MetersPerSecond
        .of(xAxis.getAsDouble() * constDrivetrain.REAL_DRIVE_SPEED.in(Units.MetersPerSecond) * redAllianceMultiplier
            - globalMotion.getLiftPosition().in(Units.Meters) / constMotion.HEIGHT_DIVIDER.in(Units.Meters));
    LinearVelocity yVelocity = Units.MetersPerSecond
        .of(-yAxis.getAsDouble() * constDrivetrain.REAL_DRIVE_SPEED.in(Units.MetersPerSecond) * redAllianceMultiplier
            - globalMotion.getLiftPosition().in(Units.Meters) / constMotion.HEIGHT_DIVIDER.in(Units.Meters));
    if (isInAutoDriveZone) {
      Pose2d closestPose = subDrivetrain.getDesiredPose(Field.getNetPositions().get());
      subDrivetrain.autoAlign(Field.isRedAlliance(),
          closestPose,
          xVelocity,
          yVelocity,
          isOpenLoop,
          false,
          true);
      subDriverStateMachine.setDriverState(DriverState.NET_AUTO_DRIVING);
    } else {
      subDrivetrain.rotationalAlign(Field.isRedAlliance(),
          subDrivetrain.getDesiredPose(Field.getNetPositions().get()),
          xVelocity,
          yVelocity,
          isOpenLoop);
      subDriverStateMachine.setDriverState(DriverState.NET_ROTATION_SNAPPING);
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
