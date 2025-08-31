package frc.robot.commands.driver_states;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.DriverStateMachine;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DriverStateMachine.DriverState;
import frc.robot.Field;

public class CageRotationSnapping extends Command {
  Drivetrain subDrivetrain;
  DriverStateMachine subDriverStateMachine;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  boolean isOpenLoop;
  double redAllianceMultiplier = 1;

  public CageRotationSnapping(Drivetrain subDrivetrain, DriverStateMachine subDriverStateMachine,
      DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier rotationAxis) {
    this.subDrivetrain = subDrivetrain;
    this.subDriverStateMachine = subDriverStateMachine;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    addRequirements(this.subDrivetrain, this.subDriverStateMachine);
    isOpenLoop = true;
  }

  @Override
  public void initialize() {
    redAllianceMultiplier = Field.isRedAlliance() ? -1 : 1;
    subDriverStateMachine.setDriverState(DriverState.CAGE_ROTATION_SNAPPING);
  }

  @Override
  public void execute() {
    LinearVelocity xVelocity = Units.MetersPerSecond
        .of(xAxis.getAsDouble() * constDrivetrain.REAL_DRIVE_SPEED.in(Units.MetersPerSecond) * redAllianceMultiplier);
    LinearVelocity yVelocity = Units.MetersPerSecond
        .of(-yAxis.getAsDouble() * constDrivetrain.REAL_DRIVE_SPEED.in(Units.MetersPerSecond) * redAllianceMultiplier);
    subDrivetrain.rotationalAlign(Field.isRedAlliance(),
        subDrivetrain.getDesiredPose(Field.getCagePositions().get()),
        xVelocity,
        yVelocity,
        isOpenLoop);
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
