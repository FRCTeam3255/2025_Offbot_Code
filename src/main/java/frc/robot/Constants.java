// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.frcteam3255.components.swerve.SN_SwerveConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {

  public static class constControllers {
    public static final double DRIVER_LEFT_STICK_DEADBAND = 0.05;
    public static final boolean SILENCE_JOYSTICK_WARNINGS = true;
  }

  public static class constDrivetrain {
    // TODO: Convert all applicable fields to MEASUREs

    // In Rotations: Obtain by aligning all of the wheels in the correct direction
    // and copy-pasting the Raw Absolute Encoder value

    // TODO: Swoffsets
    public static final double FRONT_LEFT_ABS_ENCODER_OFFSET = -0.187012;
    public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET = 0.453369;
    public static final double BACK_LEFT_ABS_ENCODER_OFFSET = 0.183350;
    public static final double BACK_RIGHT_ABS_ENCODER_OFFSET = -0.000977;

    public static final SN_SwerveConstants SWERVE_CONSTANTS = new SN_SwerveConstants(
        SN_SwerveConstants.MK4I.FALCON.L3.steerGearRatio,
        0.09779 * Math.PI,
        SN_SwerveConstants.MK4I.FALCON.L3.driveGearRatio,
        SN_SwerveConstants.MK4I.FALCON.L3.maxSpeedMeters);

    public static final double WHEEL_DIAMETER = SWERVE_CONSTANTS.wheelCircumference / Math.PI;
    public static final Distance WHEEL_RADIUS = Units.Meters.of(WHEEL_DIAMETER / 2);

    /**
     * <p>
     * Observed maximum translational speed while manually driving on the
     * Competition Robot.
     * </p>
     */
    public static final LinearVelocity REAL_DRIVE_SPEED = Units.FeetPerSecond.of(15.1);
    // Physically measured from center to center of the wheels
    // Distance between Left & Right Wheels for 25 by 25 frame
    public static final double TRACK_WIDTH_25 = Units.Meters.convertFrom(19.75, Units.Inches);
    // Distance between Front & Back Wheels for 25 by 25 frame
    public static final double WHEELBASE_25 = Units.Meters.convertFrom(19.75, Units.Inches);

    // Distance between Left & Right Wheels for 29 by 29 frame
    public static final double TRACK_WIDTH_29 = Units.Meters.convertFrom(23.75, Units.Inches);
    // Distance between Front & Back Wheels for 29 by 29 frame
    public static final double WHEELBASE_29 = Units.Meters.convertFrom(23.75, Units.Inches);

    // Distance between Left & Right Wheels
    public static final double TRACK_WIDTH = TRACK_WIDTH_29; // TODO: Replace with actual measurement
    // Distance between Front & Back Wheels
    public static final double WHEELBASE = WHEELBASE_29; // TODO: Replace with actual measurement

    // -- Pose Estimation --
    /**
     * <p>
     * Pose estimator standard deviation for encoder & gyro data
     * </p>
     * <b>Units:</b> Meters
     */
    public static final double MEASUREMENT_STD_DEVS_POS = 0.05;
    /**
     * <p>
     * Pose estimator standard deviation for encoder & gyro data
     * </p>
     * <b>Units:</b> Radians
     */
    public static final double MEASUREMENT_STD_DEV_HEADING = Units.Radians.convertFrom(5, Units.Degrees);

    // -- CONFIGS --
    public static TalonFXConfiguration DRIVE_CONFIG = new TalonFXConfiguration();
    public static TalonFXConfiguration STEER_CONFIG = new TalonFXConfiguration();
    public static CANcoderConfiguration CANCODER_CONFIG = new CANcoderConfiguration();

    // This config is kept separate as it's also used in the MODULE_CONFIG :p
    public static final Current DRIVE_CURRENT_LIMIT = Units.Amps.of(99999);

    public static final double MIN_STEER_PERCENT = 0.01;

    // Rotational speed (degrees per second) while manually driving
    public static final AngularVelocity TURN_SPEED = Units.DegreesPerSecond.of(360);

    // -- Motor Configurations --
    static {
      // This PID is implemented on each module, not the Drivetrain subsystem.
      // TODO: PID
      DRIVE_CONFIG.Slot0.kP = 0.18;
      DRIVE_CONFIG.Slot0.kI = 0.0;
      DRIVE_CONFIG.Slot0.kD = 0.0;
      DRIVE_CONFIG.Slot0.kS = 0.0;
      DRIVE_CONFIG.Slot0.kA = 0.0;
      DRIVE_CONFIG.Slot0.kV = (1 / REAL_DRIVE_SPEED.in(Units.MetersPerSecond));

      DRIVE_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      DRIVE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      DRIVE_CONFIG.Feedback.SensorToMechanismRatio = SWERVE_CONSTANTS.driveGearRatio;
      DRIVE_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      DRIVE_CONFIG.CurrentLimits.SupplyCurrentLimit = DRIVE_CURRENT_LIMIT.in(Units.Amps);

      STEER_CONFIG.Slot0.kP = 100;
      STEER_CONFIG.Slot0.kI = 0.0;
      STEER_CONFIG.Slot0.kD = 0.14414076246334312;

      STEER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      STEER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      STEER_CONFIG.Feedback.SensorToMechanismRatio = SWERVE_CONSTANTS.steerGearRatio;
      STEER_CONFIG.ClosedLoopGeneral.ContinuousWrap = true;

      CANCODER_CONFIG.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    }

    public static class AUTO {
      // This PID is implemented on the Drivetrain subsystem
      // TODO: AUTO PID
      public static final PIDConstants AUTO_DRIVE_PID = new PIDConstants(9, 0.0, 0.0);

      public static final PIDConstants AUTO_STEER_PID = new PIDConstants(5.6, 0.0, 0.0);

      // Feet
      public static final double AUTO_MAX_SPEED = 8;
      // Feet per second
      public static final double AUTO_MAX_ACCEL = 6;

      public static final Mass MASS = Units.Kilograms.of(20);
      public static final double MOI = 8;
      public static final double WHEEL_COF = 1.0;
      public static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60(1);
      public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(WHEEL_RADIUS, REAL_DRIVE_SPEED, WHEEL_COF,
          DRIVE_MOTOR,
          DRIVE_CURRENT_LIMIT, 1);

      public static final Translation2d[] MODULE_OFFSETS = {
          new Translation2d(WHEELBASE / 2.0, TRACK_WIDTH / 2.0),
          new Translation2d(WHEELBASE / 2.0, -TRACK_WIDTH / 2.0),
          new Translation2d(-WHEELBASE / 2.0, TRACK_WIDTH / 2.0),
          new Translation2d(-WHEELBASE / 2.0, -TRACK_WIDTH / 2.0) };

      public static final RobotConfig ROBOT_CONFIG = new RobotConfig(MASS.in(Kilograms), MOI, MODULE_CONFIG,
          MODULE_OFFSETS);
    }

    public static class TELEOP_AUTO_ALIGN {
      public static final LinearVelocity MIN_DRIVER_OVERRIDE = constDrivetrain.REAL_DRIVE_SPEED.div(10);

      public static final PIDController TRANS_CONTROLLER = new PIDController(
          4,
          0,
          0);
      public static final Distance AT_POINT_TOLERANCE = Units.Inches.of(0.5);

      public static final ProfiledPIDController ROTATION_CONTROLLER = new ProfiledPIDController(
          3, 0, 0, new TrapezoidProfile.Constraints(TURN_SPEED.in(Units.DegreesPerSecond),
              Math.pow(TURN_SPEED.in(Units.DegreesPerSecond), 2)));
      public static final Angle AT_ROTATION_TOLERANCE = Units.Degrees.of(1);

      static {
        TRANS_CONTROLLER.setTolerance(AT_POINT_TOLERANCE.in(Units.Meters));

        ROTATION_CONTROLLER.enableContinuousInput(0, 360);
        ROTATION_CONTROLLER.setTolerance(AT_ROTATION_TOLERANCE.in(Units.Degrees));
      }

      public static HolonomicDriveController TELEOP_AUTO_ALIGN_CONTROLLER = new HolonomicDriveController(
          TRANS_CONTROLLER,
          TRANS_CONTROLLER,
          ROTATION_CONTROLLER);
    }
  }

  public static class constMotion {
    public static TalonFXConfiguration LIFT_CONFIG = new TalonFXConfiguration();
    public static TalonFXConfiguration ELEVATOR_PIVOT_CONFIG = new TalonFXConfiguration();
    public static TalonFXConfiguration WRIST_CONFIG = new TalonFXConfiguration();

    static {
      // elevator motor config
      LIFT_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      LIFT_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      LIFT_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      LIFT_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Inches.of(62).in(Units.Meters);
      LIFT_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      LIFT_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Inches.of(0).in(Units.Meters);
      LIFT_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;
      LIFT_CONFIG.Feedback.SensorToMechanismRatio = 10;
      LIFT_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 0;
      LIFT_CONFIG.MotionMagic.MotionMagicAcceleration = 0;
      LIFT_CONFIG.MotionMagic.MotionMagicExpo_kV = 0.04;
      LIFT_CONFIG.MotionMagic.MotionMagicExpo_kA = 0.005;
      LIFT_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      LIFT_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 30;
      LIFT_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;
      LIFT_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 1;

      // elevator pivot motor config
      ELEVATOR_PIVOT_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      ELEVATOR_PIVOT_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      ELEVATOR_PIVOT_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      ELEVATOR_PIVOT_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Rotations.of(57)
          .in(Units.Degrees);
      ELEVATOR_PIVOT_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      ELEVATOR_PIVOT_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Rotations.of(-37)
          .in(Units.Degrees);

      ELEVATOR_PIVOT_CONFIG.Feedback.SensorToMechanismRatio = 102.22;

      ELEVATOR_PIVOT_CONFIG.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
      ELEVATOR_PIVOT_CONFIG.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

      ELEVATOR_PIVOT_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 9999;
      ELEVATOR_PIVOT_CONFIG.MotionMagic.MotionMagicAcceleration = 9999;

      ELEVATOR_PIVOT_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      ELEVATOR_PIVOT_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 30;
      ELEVATOR_PIVOT_CONFIG.CurrentLimits.SupplyCurrentLimit = 45;
      ELEVATOR_PIVOT_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 0.5;

      // intake pivot motor
      WRIST_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      WRIST_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      WRIST_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      WRIST_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Rotations.of(57)
          .in(Units.Degrees);
      WRIST_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      WRIST_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Rotations.of(-37)
          .in(Units.Degrees);

      WRIST_CONFIG.Feedback.SensorToMechanismRatio = 58.16;
      WRIST_CONFIG.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
      WRIST_CONFIG.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

      WRIST_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 9999;
      WRIST_CONFIG.MotionMagic.MotionMagicAcceleration = 9999;

      WRIST_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      WRIST_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 30;
      WRIST_CONFIG.CurrentLimits.SupplyCurrentLimit = 45;
      WRIST_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 0.5;
    }

    public static final Distance DEADZONE_DISTANCE = Units.Inches.of(0);
    public static final AngularVelocity ZEROED_VELOCITY = Units.RotationsPerSecond.of(0.2);

    public static final Angle MAX_POS = Units.Degrees.of(90);
    public static final Angle MIN_POS = Units.Degrees.of(0);

    public static final Angle ZEROED_MANUAL_POS = Units.Degrees.of(57);
    public static final Angle ZEROED_AUTO_POS = Units.Degrees.of(59);

    /**
     * The elapsed time required to consider the motor as zeroed
     */
    public static final Time ZEROED_TIME = Units.Seconds.of(1);

    public static final Voltage ZEROING_VOLTAGE = Units.Volts.of(1);
  }

  public static class constRotors {
    public static TalonFXConfiguration CLIMBER_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration ALGAE_INTAKE_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration CORAL_INTAKE_CONFIG = new TalonFXConfiguration();
    public static final CANrangeConfiguration CORAL_INTAKE_SENSOR_CONFIG = new CANrangeConfiguration();

    static {
      CLIMBER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      CLIMBER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      CLIMBER_CONFIG.CurrentLimits.SupplyCurrentLimit = 85;
      CLIMBER_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 60;
      CLIMBER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      // algae intake motor config
      ALGAE_INTAKE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      ALGAE_INTAKE_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      ALGAE_INTAKE_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      ALGAE_INTAKE_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 30;
      ALGAE_INTAKE_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;
      ALGAE_INTAKE_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 0.5;

      // coral intake motor config
      CORAL_INTAKE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      CORAL_INTAKE_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      CORAL_INTAKE_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      CORAL_INTAKE_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 30;
      CORAL_INTAKE_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;
      CORAL_INTAKE_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 0.5;

      // coral intake sensor config
      CORAL_INTAKE_SENSOR_CONFIG.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
      CORAL_INTAKE_SENSOR_CONFIG.ProximityParams.ProximityThreshold = Units.Inches.of(3.95).in(Units.Meters);
    }
    public static final Current ALGAE_INTAKE_HAS_GP_CURRENT = Units.Amps.of(15);
    public static final AngularVelocity ALGAE_INTAKE_HAS_GP_VELOCITY = Units.RotationsPerSecond.of(2102 / 60);
  }

  public static class MechanismPositionGroup {
    public Angle wristAngle;
    public Distance liftHeight;
    public Angle pivotAngle;
  }

  public static class constMechanismPositions {
    public static final MechanismPositionGroup CLEAN_HIGH_FORWARDS = new MechanismPositionGroup();
    public static final MechanismPositionGroup CLEAN_LOW_FORWARDS = new MechanismPositionGroup();
    public static final MechanismPositionGroup CLEAN_HIGH_BACKWARDS = new MechanismPositionGroup();
    public static final MechanismPositionGroup CLEAN_LOW_BACKWARDS = new MechanismPositionGroup();
    public static final MechanismPositionGroup INTAKE_CORAL_GROUND = new MechanismPositionGroup();
    public static final MechanismPositionGroup INTAKE_CORAL_STATION = new MechanismPositionGroup();
    public static final MechanismPositionGroup INTAKE_ALGAE_GROUND = new MechanismPositionGroup();
    public static final MechanismPositionGroup PREP_CORAL_L1 = new MechanismPositionGroup();
    public static final MechanismPositionGroup PREP_CORAL_L2_FORWARDS = new MechanismPositionGroup();
    public static final MechanismPositionGroup PREP_CORAL_L3_FORWARDS = new MechanismPositionGroup();
    public static final MechanismPositionGroup PREP_CORAL_L4_FORWARDS = new MechanismPositionGroup();
    public static final MechanismPositionGroup PREP_CORAL_L2_BACKWARDS = new MechanismPositionGroup();
    public static final MechanismPositionGroup PREP_CORAL_L3_BACKWARDS = new MechanismPositionGroup();
    public static final MechanismPositionGroup PREP_CORAL_L4_BACKWARDS = new MechanismPositionGroup();
    public static final MechanismPositionGroup PREP_CORAL_ZERO_WITH_ALGAE = new MechanismPositionGroup();
    public static final MechanismPositionGroup PREP_ALGAE_NET_FORWARDS = new MechanismPositionGroup();
    public static final MechanismPositionGroup PREP_ALGAE_NET_BACKWARDS = new MechanismPositionGroup();
    public static final MechanismPositionGroup PREP_CORAL_ZERO = new MechanismPositionGroup();
    public static final MechanismPositionGroup PREP_CLIMB = new MechanismPositionGroup();
    public static final MechanismPositionGroup NONE = new MechanismPositionGroup();
    public static final MechanismPositionGroup PREP_ALGAE_ZERO = new MechanismPositionGroup();
    public static final MechanismPositionGroup PREP_PROCESSOR = new MechanismPositionGroup();
    public static final MechanismPositionGroup CLIMBING = new MechanismPositionGroup();
    public static final Distance ELEVATOR_CORAL_L1_HEIGHT = Units.Inches.of(0);
    public static final Distance ELEVATOR_CORAL_L2_HEIGHT = Units.Inches.of(7);
    public static final Distance ELEVATOR_CORAL_L3_HEIGHT = Units.Inches.of(20);
    public static final Distance ELEVATOR_CORAL_L4_HEIGHT = Units.Inches.of(46);
    public static final Distance ELEVATOR_CLIMBING_HEIGHT = Units.Inches.of(5);
    static {

      CLEAN_HIGH_FORWARDS.wristAngle = Degrees.of(-65.81);
      CLEAN_HIGH_FORWARDS.liftHeight = Inches.of(20.975);
      CLEAN_HIGH_FORWARDS.pivotAngle = Degrees.of(69.04);

      CLEAN_LOW_FORWARDS.wristAngle = Degrees.of(-65.15);
      CLEAN_LOW_FORWARDS.liftHeight = Inches.of(10);
      CLEAN_LOW_FORWARDS.pivotAngle = Degrees.of(58.46);

      CLEAN_LOW_BACKWARDS.wristAngle = Degrees.of(98.05);
      CLEAN_LOW_BACKWARDS.liftHeight = Inches.of(5);
      CLEAN_LOW_BACKWARDS.pivotAngle = Degrees.of(83.88);

      CLEAN_HIGH_BACKWARDS.wristAngle = Degrees.of(99.26);
      CLEAN_HIGH_BACKWARDS.liftHeight = Inches.of(22);
      CLEAN_HIGH_BACKWARDS.pivotAngle = Degrees.of(86.16);

      INTAKE_CORAL_GROUND.wristAngle = Degrees.of(0); // TODO: Replace with actual angle
      INTAKE_CORAL_GROUND.liftHeight = Inches.of(0); // TODO: Replace with actual height
      INTAKE_CORAL_GROUND.pivotAngle = Degrees.of(0); // TODO: Replace with actual angle

      INTAKE_CORAL_STATION.wristAngle = Degrees.of(-10.9);
      INTAKE_CORAL_STATION.liftHeight = Inches.of(7.5);// huxly said 6.958
      INTAKE_CORAL_STATION.pivotAngle = Degrees.of(56.9);

      INTAKE_ALGAE_GROUND.wristAngle = Degrees.of(0); // TODO: Replace with actual angle
      INTAKE_ALGAE_GROUND.liftHeight = Inches.of(0); // TODO: Replace with actual height
      INTAKE_ALGAE_GROUND.pivotAngle = Degrees.of(0); // TODO: Replace with actual angle

      PREP_CORAL_L1.wristAngle = Degrees.of(-10);
      PREP_CORAL_L1.liftHeight = Inches.of(0); // TODO: Replace with actual height
      PREP_CORAL_L1.pivotAngle = Degrees.of(35);

      PREP_CORAL_L2_FORWARDS.wristAngle = Degrees.of(115);
      PREP_CORAL_L2_FORWARDS.liftHeight = Inches.of(7); // TODO: Replace with actual height
      PREP_CORAL_L2_FORWARDS.pivotAngle = Degrees.of(35.56);

      PREP_CORAL_L3_FORWARDS.wristAngle = Degrees.of(100);
      PREP_CORAL_L3_FORWARDS.liftHeight = Inches.of(19); // TODO: Replace with actual height
      PREP_CORAL_L3_FORWARDS.pivotAngle = Degrees.of(50.9);

      PREP_CORAL_L4_FORWARDS.wristAngle = Degrees.of(35);
      PREP_CORAL_L4_FORWARDS.liftHeight = Inches.of(44); // TODO: Replace with actual height
      PREP_CORAL_L4_FORWARDS.pivotAngle = Degrees.of(70.01);

      PREP_CORAL_L2_BACKWARDS.wristAngle = Degrees.of(130);
      PREP_CORAL_L2_BACKWARDS.liftHeight = Inches.of(0); // TODO: Replace with actual height
      PREP_CORAL_L2_BACKWARDS.pivotAngle = Degrees.of(86.64);

      PREP_CORAL_L3_BACKWARDS.wristAngle = Degrees.of(125);
      PREP_CORAL_L3_BACKWARDS.liftHeight = Inches.of(15); // TODO: Replace with actual height
      PREP_CORAL_L3_BACKWARDS.pivotAngle = Degrees.of(86.33);

      PREP_CORAL_L4_BACKWARDS.wristAngle = Degrees.of(145);
      PREP_CORAL_L4_BACKWARDS.liftHeight = Inches.of(44); // TODO: Replace with actual height
      PREP_CORAL_L4_BACKWARDS.pivotAngle = Degrees.of(89.65);

      PREP_CORAL_ZERO_WITH_ALGAE.wristAngle = Degrees.of(0); // TODO: Replace with actual angle
      PREP_CORAL_ZERO_WITH_ALGAE.liftHeight = Inches.of(0); // TODO: Replace with actual height
      PREP_CORAL_ZERO_WITH_ALGAE.pivotAngle = Degrees.of(0); // TODO: Replace with actual angle

      PREP_ALGAE_NET_FORWARDS.wristAngle = Degrees.of(-40); // TODO: Replace with actual angle
      PREP_ALGAE_NET_FORWARDS.liftHeight = Inches.of(50); // TODO: Replace with actual height
      PREP_ALGAE_NET_FORWARDS.pivotAngle = Degrees.of(90); // TODO: Replace with actual angle

      PREP_ALGAE_NET_BACKWARDS.wristAngle = Degrees.of(40);
      PREP_ALGAE_NET_BACKWARDS.liftHeight = Inches.of(50);
      PREP_ALGAE_NET_BACKWARDS.pivotAngle = Degrees.of(90);

      PREP_ALGAE_ZERO.wristAngle = Degrees.of(0); // TODO: Replace with actual angle
      PREP_ALGAE_ZERO.liftHeight = Inches.of(0); // TODO: Replace with actual height
      PREP_ALGAE_ZERO.pivotAngle = Degrees.of(0); // TODO: Replace with actual angle

      PREP_CORAL_ZERO.wristAngle = Degrees.of(0); // TODO: Replace with actual angle
      PREP_CORAL_ZERO.liftHeight = Inches.of(0); // TODO: Replace with actual height
      PREP_CORAL_ZERO.pivotAngle = Degrees.of(0); // TODO: Replace with actual angle

      PREP_CLIMB.wristAngle = Degrees.of(0); // TODO: Replace with actual angle
      PREP_CLIMB.liftHeight = Inches.of(0); // TODO: Replace with actual height
      PREP_CLIMB.pivotAngle = Degrees.of(90); // TODO: Replace with actual angle

      NONE.wristAngle = Degrees.of(0); // TODO: Replace with actual angle
      NONE.liftHeight = Inches.of(0); // TODO: Replace with actual height
      NONE.pivotAngle = Degrees.of(70); // TODO: Replace with actual angle

      PREP_PROCESSOR.wristAngle = Degrees.of(0); // TODO: Replace with actual angle
      PREP_PROCESSOR.liftHeight = Inches.of(0); // TODO: Replace with actual height
      PREP_PROCESSOR.pivotAngle = Degrees.of(10); // TODO: Replace with actual angle

      CLIMBING.wristAngle = Degrees.of(0); // TODO: Replace with actual angle
      CLIMBING.liftHeight = Inches.of(0); // TODO: Replace with actual height
      CLIMBING.pivotAngle = Degrees.of(0); // TODO: Replace with actual angle
      // backwards positions

    }
  }

  public static class constRotorsSpeeds {
    // change all elevator/intake and their related methods to be Motion/Rotors and
    // setAllPosition() in Motion.java with groups in Constants.java ; change all
    // set speeds to methods in Rotors.java with constants in constRotorSpeeds
    public static final double EJECTING_GAME_PIECE_SPEED = 0; // TODO: Replace with actual speed
    // algae speed consts
    public static final double INTAKE_ALGAE_SPEED = 0; // TODO: Replace with actual speed
    public static final double SCORE_ALGAE_NET_SPEED = 0; // TODO: Replace with actual speed
    public static final double SCORE_ALGAE_PROCESSOR_SPEED = 0; // TODO: Replace with actual speed
    public static final double CLIMBER_MOTOR_PERCENT_OUTPUT = 1;
    public static final double CLEAN_ALGAE_SPEED = 0;
    public static final double INTAKE_CORAL_GROUND_SPEED = 0; // TODO: Replace with actual speed
    public static final double INTAKE_CORAL_STATION_SPEED = 0; // TODO: Replace with actual speed
    public static final double SCORE_CORAL_SPEED = 0;
  }

  /**
   * Volts
   */
  public static final double MAX_VOLTAGE = 12;

  public static final Transform3d ROBOT_TO_BUMPERS = new Transform3d(0, 0, Units.Meters.convertFrom(5, Units.Inches),
      Rotation3d.kZero); // TODO: Replace with actual measurement

  public static class constField {
    public static Optional<Alliance> ALLIANCE = Optional.empty();
    public static final Distance FIELD_LENGTH = Units.Feet.of(57).plus(Units.Inches.of(6 + 7 / 8));
    public static final Distance FIELD_WIDTH = Units.Feet.of(26).plus(Units.Inches.of(5));

    public static final Distance REEF_AUTO_DRIVE_MAX_DISTANCE = Units.Meters.of(3);
    public static final Distance CORAL_STATION_AUTO_DRIVE_MAX_DISTANCE = Units.Meters.of(5);
    public static final Distance ALGAE_AUTO_DRIVE_MAX_DISTANCE = Units.Meters.of(3);
    public static final Distance PROCESSOR_AUTO_DRIVE_MAX_DISTANCE = Units.Meters.of(5);
    public static final Distance NET_AUTO_DRIVE_MAX_DISTANCE = Units.Meters.of(5);

    /**
     * Boolean that controls when the path will be mirrored for the red
     * alliance. This will flip the path being followed to the red side of the
     * field.
     * The origin will remain on the Blue side.
     * 
     * @return If we are currently on Red alliance. Will return false if no alliance
     *         is found
     */
    public static boolean isRedAlliance() {
      var alliance = ALLIANCE;
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    };

    public static class POSES {
      public static final Pose2d RESET_POSE = new Pose2d(3.169, 4.015, new Rotation2d());
      public static final Pose3d SCORING_ELEMENT_NOT_COLLECTED = new Pose3d(0, 0, -1, Rotation3d.kZero);

      // Branch Poses
      private static final Pose2d REEF_A = new Pose2d(3.171, 4.189, Rotation2d.kZero);
      private static final Pose2d REEF_B = new Pose2d(3.171, 3.863, Rotation2d.kZero);
      private static final Pose2d REEF_C = new Pose2d(3.688, 2.968, Rotation2d.fromDegrees(60));
      private static final Pose2d REEF_D = new Pose2d(3.975, 2.803, Rotation2d.fromDegrees(60));
      private static final Pose2d REEF_E = new Pose2d(5.001, 2.804, Rotation2d.fromDegrees(120));
      private static final Pose2d REEF_F = new Pose2d(5.285, 2.964, Rotation2d.fromDegrees(120));
      private static final Pose2d REEF_G = new Pose2d(5.805, 3.863, Rotation2d.k180deg);
      private static final Pose2d REEF_H = new Pose2d(5.805, 4.189, Rotation2d.k180deg);
      private static final Pose2d REEF_I = new Pose2d(5.288, 5.083, Rotation2d.fromDegrees(-120));
      private static final Pose2d REEF_J = new Pose2d(5.002, 5.248, Rotation2d.fromDegrees(-120));
      private static final Pose2d REEF_K = new Pose2d(3.972, 5.247, Rotation2d.fromDegrees(-60));
      private static final Pose2d REEF_L = new Pose2d(3.693, 5.079, Rotation2d.fromDegrees(-60));

      // Net Poses
      private static final Pose2d CENTER_LINE = new Pose2d(Units.Meters.of(8.850), Units.Meters.of(6.174),
          Rotation2d.fromDegrees(0));

      // algae poses
      public static final Pose2d ALGAE_AB = REEF_A.interpolate(REEF_B, 0.5);
      public static final Pose2d ALGAE_CD = REEF_C.interpolate(REEF_D, 0.5);
      public static final Pose2d ALGAE_EF = REEF_E.interpolate(REEF_F, 0.5);
      public static final Pose2d ALGAE_GH = REEF_G.interpolate(REEF_H, 0.5);
      public static final Pose2d ALGAE_IJ = REEF_I.interpolate(REEF_J, 0.5);
      public static final Pose2d ALGAE_KL = REEF_K.interpolate(REEF_L, 0.5);

      // Coral Station
      private static final Pose2d LEFT_CORAL_STATION_FAR = new Pose2d(1.64, 7.33, Rotation2d.fromDegrees(-55));
      private static final Pose2d LEFT_CORAL_STATION_NEAR = new Pose2d(0.71, 6.68, Rotation2d.fromDegrees(-55));
      private static final Pose2d RIGHT_CORAL_STATION_FAR = new Pose2d(1.61, 0.70, Rotation2d.fromDegrees(55));
      private static final Pose2d RIGHT_CORAL_STATION_NEAR = new Pose2d(0.64, 1.37, Rotation2d.fromDegrees(55));

      // Processor
      private static final Pose2d BLUE_PROCESSOR = new Pose2d(5.6, 0.896, Rotation2d.fromDegrees(-90));

      private static final Pose2d RED_PROCESSOR = getRedAlliancePose(BLUE_PROCESSOR);

      // Cages
      public static final Pose2d CAGE_1 = new Pose2d(7.783, 7.248, Rotation2d.fromDegrees(180));
      public static final Pose2d CAGE_2 = new Pose2d(7.783, 6.151, Rotation2d.fromDegrees(180));
      public static final Pose2d CAGE_3 = new Pose2d(7.783, 5.068, Rotation2d.fromDegrees(180));

      // --- lists ---
      private static final List<Pose2d> BLUE_NET_POSES = List.of(CENTER_LINE);
      private static final List<Pose2d> RED_NET_POSES = getRedNetPoses();

      // -- REEF --
      private static final List<Pose2d> BLUE_REEF_POSES = List.of(REEF_A, REEF_B, REEF_C, REEF_D, REEF_E,
          REEF_F, REEF_G, REEF_H, REEF_I, REEF_J, REEF_K, REEF_L);
      private static final List<Pose2d> RED_REEF_POSES = getRedReefPoses();
      private static final List<Pose2d> BLUE_REEF_BACKWARDS_SCORING_POSES = getBlueReefBackwardsScoringPoses();
      private static final List<Pose2d> RED_REEF_BACKWARDS_SCORING_POSES = getRedReefBackwardsScoringPoses(
          BLUE_REEF_BACKWARDS_SCORING_POSES);
      private static final List<Pose2d> BLUE_LEFT_REEF_POSES = List.of(REEF_A, REEF_C, REEF_F, REEF_H, REEF_J,
          REEF_K);
      private static final List<Pose2d> RED_LEFT_REEF_POSES = List.of(getRedPosesFromList(BLUE_LEFT_REEF_POSES));
      private static final List<Pose2d> BLUE_RIGHT_REEF_POSES = List.of(REEF_B, REEF_D, REEF_E, REEF_G, REEF_I,
          REEF_L);
      private static final List<Pose2d> RED_RIGHT_REEF_POSES = List.of(getRedPosesFromList(BLUE_RIGHT_REEF_POSES));

      // -- Algae --
      private static final List<Pose2d> BLUE_ALGAE_POSES = List.of(ALGAE_AB, ALGAE_CD, ALGAE_EF, ALGAE_GH, ALGAE_IJ,
          ALGAE_KL);
      private static final List<Pose2d> RED_ALGAE_POSES = getRedAlgaePoses();

      // -- Coral Station --
      private static final List<Pose2d> BLUE_CORAL_STATION_POSES = List.of(LEFT_CORAL_STATION_FAR,
          LEFT_CORAL_STATION_NEAR, RIGHT_CORAL_STATION_FAR, RIGHT_CORAL_STATION_NEAR);
      private static final List<Pose2d> RED_CORAL_STATION_POSES = getRedCoralStationPoses();
      // -- Cage --
      private static final List<Pose2d> BLUE_CAGE_POSES = List.of(CAGE_1, CAGE_2, CAGE_3);
      private static final List<Pose2d> RED_CAGE_POSES = getRedCagePoses();
    }

    public static Pose2d getRedAlliancePose(Pose2d bluePose) {
      return new Pose2d(FIELD_LENGTH.in(Units.Meters) - (bluePose.getX()),
          FIELD_WIDTH.in(Units.Meters) - bluePose.getY(),
          bluePose.getRotation().plus(Rotation2d.k180deg));
    }

    private static Pose2d[] getRedPosesFromList(List<Pose2d> bluePoseList) {
      Pose2d[] returnedPoses = new Pose2d[bluePoseList.size()];
      for (int i = 0; i < bluePoseList.size(); i++) {
        returnedPoses[i] = getRedAlliancePose(bluePoseList.get(i));
      }
      return returnedPoses;
    }

    private static Pose2d getBackwardsScoringPoses(Pose2d bluePose) {
      return new Pose2d(bluePose.getX(), bluePose.getY(),
          bluePose.getRotation().plus(Rotation2d.k180deg));
    }

    private static Pose2d[] getBackwardsScoringPosesFromList(List<Pose2d> bluePoseList) {
      Pose2d[] returnedPoses = new Pose2d[bluePoseList.size()];
      for (int i = 0; i < bluePoseList.size(); i++) {
        returnedPoses[i] = getBackwardsScoringPoses(bluePoseList.get(i));
      }
      return returnedPoses;
    }

    // -- REEF --
    private static List<Pose2d> getRedReefPoses() {
      Pose2d[] returnedPoses = getRedPosesFromList(POSES.BLUE_REEF_POSES);
      return List.of(returnedPoses[0], returnedPoses[1], returnedPoses[2], returnedPoses[3], returnedPoses[4],
          returnedPoses[5], returnedPoses[6], returnedPoses[7], returnedPoses[8], returnedPoses[9], returnedPoses[10],
          returnedPoses[11]);
    }

    public static Supplier<List<Pose2d>> getReefPositions(boolean onRed) {
      if (onRed) {
        return () -> POSES.RED_REEF_POSES;

      }
      return () -> POSES.BLUE_REEF_POSES;
    }

    public static List<Pose2d> getBlueReefBackwardsScoringPoses() {
      Pose2d[] returnedPoses = getBackwardsScoringPosesFromList(POSES.BLUE_REEF_POSES);
      return List.of(returnedPoses[0], returnedPoses[1], returnedPoses[2], returnedPoses[3], returnedPoses[4],
          returnedPoses[5], returnedPoses[6], returnedPoses[7], returnedPoses[8], returnedPoses[9], returnedPoses[10],
          returnedPoses[11]);
    }

    private static List<Pose2d> getRedReefBackwardsScoringPoses(List<Pose2d> blueReefBackwardsScoringPoses) {
      Pose2d[] returnedPoses = new Pose2d[blueReefBackwardsScoringPoses.size()];

      for (int i = 0; i < blueReefBackwardsScoringPoses.size(); i++) {
        returnedPoses[i] = getBackwardsScoringPoses(blueReefBackwardsScoringPoses.get(i));
      }

      return List.of(returnedPoses[0], returnedPoses[1], returnedPoses[2], returnedPoses[3], returnedPoses[4],
          returnedPoses[5], returnedPoses[6], returnedPoses[7], returnedPoses[8], returnedPoses[9], returnedPoses[10],
          returnedPoses[11]);
    }

    public static Supplier<List<Pose2d>> getReefBackwardsScoringPositions(boolean onRed) {
      if (onRed) {
        return () -> POSES.RED_REEF_BACKWARDS_SCORING_POSES;
      }
      return () -> POSES.BLUE_REEF_BACKWARDS_SCORING_POSES;
    }

    public static Supplier<List<Pose2d>> getLeftReefPositions(boolean onRed) {
      if (onRed) {
        return () -> POSES.RED_LEFT_REEF_POSES;
      }
      return () -> POSES.BLUE_LEFT_REEF_POSES;
    }

    public static Supplier<List<Pose2d>> getRightReefPositions(boolean onRed) {
      if (onRed) {
        return () -> POSES.RED_RIGHT_REEF_POSES;
      }
      return () -> POSES.BLUE_RIGHT_REEF_POSES;
    }

    // -- ALGAE --
    private static List<Pose2d> getRedAlgaePoses() {
      Pose2d[] returnedPoses = new Pose2d[POSES.BLUE_ALGAE_POSES.size()];

      for (int i = 0; i < POSES.BLUE_ALGAE_POSES.size(); i++) {
        returnedPoses[i] = getRedAlliancePose(POSES.BLUE_ALGAE_POSES.get(i));
      }

      return List.of(returnedPoses[0], returnedPoses[1], returnedPoses[2], returnedPoses[3], returnedPoses[4],
          returnedPoses[5]);
    }

    public static Supplier<List<Pose2d>> getAlgaePositions(Boolean onRed) {
      if (onRed) {
        return () -> POSES.RED_ALGAE_POSES;
      }
      return () -> POSES.BLUE_ALGAE_POSES;
    }

    // -- NET --
    private static List<Pose2d> getRedNetPoses() {
      Pose2d[] returnedPoses = getRedPosesFromList(POSES.BLUE_NET_POSES);
      return List.of(returnedPoses[0]);

    }

    public static Supplier<List<Pose2d>> getNetPositions(boolean onRed) {
      if (onRed) {
        return () -> POSES.RED_NET_POSES;
      }
      return () -> POSES.BLUE_NET_POSES;
    }

    // -- CORAL STATION --
    private static List<Pose2d> getRedCoralStationPoses() {
      Pose2d[] returnedPoses = new Pose2d[POSES.BLUE_CORAL_STATION_POSES.size()];

      for (int i = 0; i < POSES.BLUE_CORAL_STATION_POSES.size(); i++) {
        returnedPoses[i] = getRedAlliancePose(POSES.BLUE_CORAL_STATION_POSES.get(i));
      }

      return List.of(returnedPoses[0], returnedPoses[1], returnedPoses[2], returnedPoses[3]);
    }

    public static Supplier<List<Pose2d>> getCoralStationPositions(boolean onRed) {
      if (onRed) {
        return () -> POSES.RED_CORAL_STATION_POSES;
      }
      return () -> POSES.BLUE_CORAL_STATION_POSES;
    }

    // -- CAGES --
    private static List<Pose2d> getRedCagePoses() {
      Pose2d[] returnedPoses = getRedPosesFromList(POSES.BLUE_CAGE_POSES);
      return List.of(returnedPoses[0], returnedPoses[1], returnedPoses[2]);
    }

    public static Supplier<List<Pose2d>> getCagePositions(boolean onRed) {
      if (onRed) {
        return () -> POSES.RED_CAGE_POSES;
      }
      return () -> POSES.BLUE_CAGE_POSES;
    }

    // -- PROCESSOR --
    public static Pose2d getProcessorPose(boolean onRed) {
      if (onRed) {
        return POSES.RED_PROCESSOR;
      }
      return POSES.BLUE_PROCESSOR;
    }

    public static final Pose2d WORKSHOP_STARTING_POSE = new Pose2d(5.98, 2.60, new Rotation2d(0));
  }

  public static class constVision {
    public static final String[] LIMELIGHT_NAMES = new String[] { "limelight-right", "limelight-left",
        "limelight-back" };

    /**
     * <p>
     * Pose estimator standard deviation for vision data
     * <p>
     * <b>Units:</b> Meters
     */

    public static final double STD_DEVS_POS = 0.7;
    /**
     * <p>
     * Pose estimator standard deviation for vision data
     * </p>
     * <b>Units:</b> Radians
     */

    public static final double STD_DEVS_HEADING = 9999999;

    /**
     * <p>
     * Pose estimator standard deviation for vision data
     * </p>
     * <b>Units:</b> Meters
     */
    public static final double MEGA_TAG1_STD_DEVS_POSITION = .3;

    public static final double MEGA_TAG1_STD_DEVS_HEADING = .1;
    /**
     * <p>
     * Maximum rate of rotation before we begin rejecting pose updates
     * </p>
     */
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = Units.DegreesPerSecond.of(720);

    /**
     * The area that one tag (if its the only tag in the update) needs to exceed
     * before being accepted
     */
    public static final double AREA_THRESHOLD_FRONT = 0.1;
    public static final double AREA_THRESHOLD_BACK = 0.05;

    // The below values are accounted for in the limelight interface, NOT in code
    public static class LIMELIGHT_RIGHT {
      public static final Distance LL_FORWARD = Units.Meters.of(0.269494);
      public static final Distance LL_RIGHT = Units.Meters.of(0.307594);
      public static final Distance LL_UP = Units.Meters.of(0.211328);

      public static final Angle LL_ROLL = Units.Degrees.of(180);
      public static final Angle LL_PITCH = Units.Degrees.of(23.17);
      public static final Angle LL_YAW = Units.Degrees.of(51.25);
    }

    public static class LIMELIGHT_LEFT {
      public static final Distance LL_FORWARD = Units.Meters.of(0.269494);
      public static final Distance LL_RIGHT = Units.Meters.of(-0.307594);
      public static final Distance LL_UP = Units.Meters.of(0.211328);

      public static final Angle LL_ROLL = Units.Degrees.of(180);
      public static final Angle LL_PITCH = Units.Degrees.of(23.17);
      public static final Angle LL_YAW = Units.Degrees.of(-51.25);

    }
  }

}
