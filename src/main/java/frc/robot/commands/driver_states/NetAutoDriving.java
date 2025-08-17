package frc.robot.commands.driver_states;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constField;
import frc.robot.subsystems.DriverStateMachine;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DriverStateMachine.DriverState;

public class NetAutoDriving extends Command {
  Drivetrain subDrivetrain;
  DriverStateMachine subDriverStateMachine;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  boolean isOpenLoop;
  double redAllianceMultiplier = 1;

  public NetAutoDriving(Drivetrain subDrivetrain, DriverStateMachine subDriverStateMachine,
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
    redAllianceMultiplier = constField.isRedAlliance() ? -1 : 1;
    subDriverStateMachine.setDriverState(DriverState.NET_AUTO_DRIVING);
  }

  @Override
  public void execute() {
    LinearVelocity xVelocity = Units.MetersPerSecond.of(xAxis.getAsDouble());
    LinearVelocity yVelocity = Units.MetersPerSecond.of(-yAxis.getAsDouble());
    Pose2d closestPose = subDrivetrain.getDesiredPose(constField.getNetPositions(constField.isRedAlliance()).get());
    subDrivetrain.autoAlign(constField.isRedAlliance(),
        closestPose,
        xVelocity,
        yVelocity,
        isOpenLoop,
        false,
        true);
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
