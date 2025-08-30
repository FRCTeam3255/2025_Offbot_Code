// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismPositionGroup;
import frc.robot.Constants.constMotion;
import frc.robot.RobotMap.mapMotion;

@Logged
public class Motion extends SubsystemBase {
  /** Creates a new Motion. */
  TalonFX leftLiftMotorFollower;
  TalonFX rightLiftMotorLeader;
  TalonFX leftPivotMotorFollower;
  TalonFX rightPivotMotorLeader;
  TalonFX wristPivotMotor;

  public boolean attemptingZeroing = false;
  public boolean hasLiftZeroed = false;
  public boolean hasPivotZeroed = false;
  public boolean hasWristZeroed = false;

  // Expose mechanisms for commands: Motion.Lift.setPosition(...)
  public final LinearMechanism Lift;
  public final AngularMechanism Pivot;
  public final AngularMechanism Wrist;
  final DCMotorSim m_motorSimModel;

  public Motion() {
    leftLiftMotorFollower = new TalonFX(mapMotion.LEFT_LIFT_CAN);
    rightLiftMotorLeader = new TalonFX(mapMotion.RIGHT_LIFT_CAN);
    leftPivotMotorFollower = new TalonFX(mapMotion.LEFT_PIVOT_CAN);
    rightPivotMotorLeader = new TalonFX(mapMotion.RIGHT_PIVOT_CAN);
    wristPivotMotor = new TalonFX(mapMotion.INTAKE_PIVOT_CAN);

    leftLiftMotorFollower.getConfigurator().apply(constMotion.LIFT_CONFIG);
    rightLiftMotorLeader.getConfigurator().apply(constMotion.LIFT_CONFIG);
    leftPivotMotorFollower.getConfigurator().apply(constMotion.ELEVATOR_PIVOT_CONFIG);
    rightPivotMotorLeader.getConfigurator().apply(constMotion.ELEVATOR_PIVOT_CONFIG);
    wristPivotMotor.getConfigurator().apply(constMotion.WRIST_CONFIG);

    // Configure mechanisms (followers set inside constructors)
    Lift = new LinearMechanism(rightLiftMotorLeader, leftLiftMotorFollower);
    Pivot = new AngularMechanism(rightPivotMotorLeader, leftPivotMotorFollower);
    Wrist = new AngularMechanism(wristPivotMotor, null);

    final double kGearRatio = 10.0;
    var elevatorSystem = LinearSystemId.createElevatorSystem(DCMotor.getFalcon500(2), 18.14, 0.091, kGearRatio);
    m_motorSimModel = new DCMotorSim(elevatorSystem, DCMotor.getKrakenX60Foc(2));

  }

  // Generic mechanisms (no subclassing required)
  @Logged
  public final class LinearMechanism {
    private final TalonFX leader, follower; // follower may be null
    private Distance lastDesired = Units.Inches.zero();
    private final MotionMagicExpoVoltage positionRequest = new MotionMagicExpoVoltage(0);

    public LinearMechanism(TalonFX leader, TalonFX follower) {
      this.leader = leader;
      this.follower = follower;
      if (follower != null)
        follower.setControl(new Follower(leader.getDeviceID(), true));
    }

    private void setPosition(Distance height) {
      leader.setControl(positionRequest.withPosition(height.in(Inches)));
      lastDesired = height;
    }

    public Distance getPosition() {

      return Units.Inches.of(leader.getPosition().getValueAsDouble());
    }

    // CHANGED: make velocity helpers public so callers can use Motion.Lift.*
    public AngularVelocity getVelocity() {
      return leader.getRotorVelocity().getValue();
    }

    public boolean isVelocityZero() {
      return getVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
    }

    public void setCoastMode(boolean coast) {
      NeutralModeValue mode = coast ? NeutralModeValue.Coast : NeutralModeValue.Brake;
      leader.setNeutralMode(mode);
      if (follower != null)
        follower.setNeutralMode(mode);
    }

    public void resetSensorPosition(Distance setpoint) {
      double v = setpoint.in(Inches);
      leader.setPosition(v);
      if (follower != null)
        follower.setPosition(v);
    }
  }

  @Logged
  public final class AngularMechanism {
    private final TalonFX leader, follower; // follower may be null
    private Angle lastDesired = Degrees.zero();
    private final MotionMagicExpoVoltage positionRequest = new MotionMagicExpoVoltage(0);

    public AngularMechanism(TalonFX leader, TalonFX follower) {
      this.leader = leader;
      this.follower = follower;
      if (follower != null)
        follower.setControl(new Follower(leader.getDeviceID(), true));
    }

    public void setPosition(Angle angle) {
      leader.setControl(positionRequest.withPosition(angle.in(Degrees)));
      lastDesired = angle;
    }

    public Angle getPosition() {

      return leader.getPosition().getValue();
    }

    public AngularVelocity getVelocity() {
      return leader.getRotorVelocity().getValue();
    }

    public boolean isVelocityZero() {
      return getVelocity().isNear(Units.RotationsPerSecond.zero(), 0.01);
    }

    public void setCoastMode(boolean coast) {
      NeutralModeValue mode = coast ? NeutralModeValue.Coast : NeutralModeValue.Brake;
      leader.setNeutralMode(mode);
      if (follower != null)
        follower.setNeutralMode(mode);
    }

    public void resetSensorPosition(Angle setpoint) {
      double v = setpoint.in(Degrees);
      leader.setPosition(v);
      if (follower != null)
        follower.setPosition(v);
    }
  }

  public void setAllPosition(MechanismPositionGroup positionGroup) {
    Lift.setPosition(positionGroup.liftHeight);
    Pivot.setPosition(positionGroup.pivotAngle);
    Wrist.setPosition(positionGroup.wristAngle);
  }

  public boolean arePositionsAtSetPoint(MechanismPositionGroup positionGroup) {
    return withinDistance(Lift.getPosition(), positionGroup.liftHeight, positionGroup.liftTolerance)
        && withinAngle(Pivot.getPosition(), positionGroup.pivotAngle, positionGroup.pivotTolerance)
        && withinAngle(Wrist.getPosition(), positionGroup.wristAngle, positionGroup.wristTolerance);
  }

  // Tolerance helpers
  private boolean withinDistance(Distance current, Distance target, Distance tol) {
    return current.compareTo(target.minus(tol)) > 0 && current.compareTo(target.plus(tol)) < 0;
  }

  private boolean withinAngle(Angle current, Angle target, Angle tol) {
    return current.compareTo(target.minus(tol)) > 0 && current.compareTo(target.plus(tol)) < 0;
  }

  public void simulationPeriodic(TalonFX m_talonFX, DCMotorSim m_motorSimModel, double kGearRatio) {
    var talonFXSim = m_talonFX.getSimState();

    // set the supply voltage of the TalonFX
    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    var motorVoltage = talonFXSim.getMotorVoltageMeasure();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
    m_motorSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(kGearRatio));
    talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(kGearRatio));
  }

  @Override
  public void periodic() {
    simulationPeriodic(Lift.follower, m_motorSimModel, 10);
    //
  }
}
