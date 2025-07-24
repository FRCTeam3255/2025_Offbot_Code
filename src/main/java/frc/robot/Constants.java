// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Kilograms;

import java.util.Optional;

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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {
  /**
   * Volts
   */
  public static final double MAX_VOLTAGE = 12;

  public static final Transform3d ROBOT_TO_BUMPERS = new Transform3d(0, 0, Units.Meters.convertFrom(5, Units.Inches),
      Rotation3d.kZero); // TODO: Replace with actual measurement

  public static class constControllers {
    public static final double DRIVER_LEFT_STICK_DEADBAND = 0.05;
    public static final boolean SILENCE_JOYSTICK_WARNINGS = true;
  }

  public static class constDrivetrain {
    // TODO: Convert all applicable fields to MEASUREs

    // In Rotations: Obtain by aligning all of the wheels in the correct direction
    // and copy-pasting the Raw Absolute Encoder value

    // TODO: Swoffsets
    public static final double FRONT_LEFT_ABS_ENCODER_OFFSET = 0.417236;
    public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET = -0.254395;
    public static final double BACK_LEFT_ABS_ENCODER_OFFSET = 0.258789;
    public static final double BACK_RIGHT_ABS_ENCODER_OFFSET = -0.290039;

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

  public static class constField {
    public static Optional<Alliance> ALLIANCE = Optional.empty();

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

    public static final Pose2d WORKSHOP_STARTING_POSE = new Pose2d(5.98, 2.60, new Rotation2d(0));
  }

  public static class constElevator {
    public static final Distance NORMAL_ELEVATOR_REVERSE_LIMIT = Units.Inches.of(0);
    public static final Distance NORMAL_ELEVATOR_FORWARD_LIMIT = Units.Inches.of(62);

    public static final Angle NORMAL_ELEVATOR_PIVOT_REVERSE_LIMIT = Units.Degrees.of(0);
    public static final Angle NORMAL_ELEVATOR_PIVOT_FORWARD_LIMIT = Units.Degrees.of(0);

    public static TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
    static {
      ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = NORMAL_ELEVATOR_FORWARD_LIMIT.in(Units.Inches);
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = NORMAL_ELEVATOR_REVERSE_LIMIT.in(Units.Inches);

      ELEVATOR_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;
      // Elevator motors will provide feedback in INCHES the carriage has moved
      ELEVATOR_CONFIG.Feedback.SensorToMechanismRatio = 0.876;

      ELEVATOR_CONFIG.Slot0.kG = 0.0; // Volts to overcome gravity
      ELEVATOR_CONFIG.Slot0.kS = 0.0; // Volts to overcome static friction
      ELEVATOR_CONFIG.Slot0.kV = 0.0; // Volts for a velocity target of 1 rps
      ELEVATOR_CONFIG.Slot0.kA = 0.0; // Volts for an acceleration of 1 rps/
      ELEVATOR_CONFIG.Slot0.kP = 0.0;
      ELEVATOR_CONFIG.Slot0.kI = 0.0;
      ELEVATOR_CONFIG.Slot0.kD = 0.0;
      ELEVATOR_CONFIG.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

      ELEVATOR_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 0;
      ELEVATOR_CONFIG.MotionMagic.MotionMagicAcceleration = 0;
      ELEVATOR_CONFIG.MotionMagic.MotionMagicExpo_kV = 0.0;
      ELEVATOR_CONFIG.MotionMagic.MotionMagicExpo_kA = 0.0;

      ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 30;
      ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;
      ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 1;

    }

    public static TalonFXConfiguration ELEVATOR_PIVOT_CONFIG = new TalonFXConfiguration();
    static {
      ELEVATOR_PIVOT_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      ELEVATOR_PIVOT_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      ELEVATOR_PIVOT_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      ELEVATOR_PIVOT_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = NORMAL_ELEVATOR_PIVOT_FORWARD_LIMIT
          .in(Units.Degrees);
      ELEVATOR_PIVOT_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      ELEVATOR_PIVOT_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = NORMAL_ELEVATOR_PIVOT_REVERSE_LIMIT
          .in(Units.Degrees);

      ELEVATOR_PIVOT_CONFIG.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
      // Elevator Pivot motors will provide feedback in Degrees the carriage has moved
      ELEVATOR_PIVOT_CONFIG.Feedback.SensorToMechanismRatio = 0.876;

      ELEVATOR_PIVOT_CONFIG.Slot0.kG = 0.0; // Volts to overcome gravity
      ELEVATOR_PIVOT_CONFIG.Slot0.kS = 0.0; // Volts to overcome static friction
      ELEVATOR_PIVOT_CONFIG.Slot0.kV = 0.0; // Volts for a velocity target of 1 rps
      ELEVATOR_PIVOT_CONFIG.Slot0.kA = 0.0; // Volts for an acceleration of 1 rps/
      ELEVATOR_PIVOT_CONFIG.Slot0.kP = 0.0;
      ELEVATOR_PIVOT_CONFIG.Slot0.kI = 0.0;
      ELEVATOR_PIVOT_CONFIG.Slot0.kD = 0.0;
      ELEVATOR_PIVOT_CONFIG.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

      ELEVATOR_PIVOT_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 0;
      ELEVATOR_PIVOT_CONFIG.MotionMagic.MotionMagicAcceleration = 0;
      ELEVATOR_PIVOT_CONFIG.MotionMagic.MotionMagicExpo_kV = 0.0;
      ELEVATOR_PIVOT_CONFIG.MotionMagic.MotionMagicExpo_kA = 0.0;

      ELEVATOR_PIVOT_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      ELEVATOR_PIVOT_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 30;
      ELEVATOR_PIVOT_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;
      ELEVATOR_PIVOT_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 1;

    }

    public static TalonFXConfiguration COAST_MODE_CONFIGURATION = new TalonFXConfiguration();
    static {
      COAST_MODE_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      COAST_MODE_CONFIGURATION.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }

    public static final Distance CORAL_L1_HEIGHT = Units.Inches.of(0);
    public static final Distance CORAL_L2_HEIGHT = Units.Inches.of(0);
    public static final Distance CORAL_L3_HEIGHT = Units.Inches.of(0);
    public static final Distance CORAL_L4_HEIGHT = Units.Inches.of(0);
    public static final Distance ALGAE_PREP_NET = Units.Inches.of(0);
    public static final Distance ALGAE_PREP_PROCESSOR_HEIGHT = Units.Inches.of(0);
    public static final Distance ALGAE_L3_CLEANING = Units.Inches.of(0);
    public static final Distance ALGAE_L2_CLEANING = Units.Inches.of(0);
    public static final Distance ALGAE_GROUND_INTAKE = Units.Inches.of(0);
    public static final Distance PREP_0 = Units.Inches.of(0);
    public static final Distance DEADZONE_DISTANCE = Units.Inches.of(0);
    public static final Distance NET_TOLERANCE = Units.Inches.of(0); // phr :)
    public static final Distance EJECT_DEADZONE = Units.Inches.of(0);
    public static final Distance CORAL_INTAKE_HIGHT = Units.Inches.of(0);
    public static final Distance INIT_TIP_HEIGHT = Units.Inches.of(0);
    public static final Distance AFTER_L1_HEIGHT = Units.Inches.of(0);
    public static final Distance EJECT_HOPPER_HEIGHT = Units.Inches.of(0);
    public static final Distance MAX_HEIGHT = Units.Inches.of(0);
    public static final Distance SAFE_TO_SLIDE = Units.Inches.of(32.55);

    public static final Angle CORAL_L1_ANGLE = Units.Degrees.of(0);
    public static final Angle CORAL_L2_ANGLE = Units.Degrees.of(0);
    public static final Angle CORAL_L3_ANGLE = Units.Degrees.of(0);
    public static final Angle CORAL_L4_ANGLE = Units.Degrees.of(0);
    public static final Angle ALGAE_PREP_NET_ANGLE = Units.Degrees.of(0);
    public static final Angle DEADZONE_ANGLE = Units.Degrees.of(2);

    public static final Distance CORAL_STUCK_OFFSET = Units.Inches.of(0);
    public static final Distance CORAL_STUCK_REVERSE_LIMIT = NORMAL_ELEVATOR_REVERSE_LIMIT.plus(CORAL_STUCK_OFFSET);

    public static final Time ZEROING_TIMEOUT = Units.Seconds.of(3);

    public static final AngularVelocity MANUAL_ZEROING_START_VELOCITY = Units.RotationsPerSecond.of(5);
    public static final AngularVelocity MANUAL_ZEROING_DELTA_VELOCITY = Units.RotationsPerSecond.of(5);

    /**
     * The voltage supplied to the motor in order to zero
     */
    public static final Voltage ZEROING_VOLTAGE = Units.Volts.of(-1);

    /**
     * The value that the motor reports when it is at it's zeroed position. This
     * may not necessarily be 0 due to mechanical slop
     */
    public static final Distance ZEROED_POS = Units.Meters.of(0);

    /**
     * The velocity that the motor goes at once it has zeroed (and can no longer
     * continue in that direction)
     */
    public static final AngularVelocity ZEROED_VELOCITY = Units.RotationsPerSecond.of(0.2);

    /**
     * The elapsed time required to consider the motor as zeroed
     */
    public static final Time ZEROED_TIME = Units.Seconds.of(1);

    public static final Transform3d CARRIAGE_TO_CORAL = new Transform3d(
        Units.Meters.convertFrom(194, Units.Millimeters), 0,
        Units.Meters.convertFrom(318 + 40, Units.Millimeters),
        new Rotation3d(0, Units.Radians.convertFrom(35, Units.Degrees), 0));
  }

  public static class constIntake {

    public static final Current ALGAE_INTAKE_HAS_GP_CURRENT = Units.Amps.of(15);
    public static final AngularVelocity ALGAE_INTAKE_HAS_GP_VELOCITY = Units.RotationsPerSecond.of(2102 / 60);
    public static final Angle MAX_POS = Units.Degrees.of(57);
    public static final Angle MIN_POS = Units.Degrees.of(-37);
    public static final TalonFXConfiguration ALGAE_INTAKE_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration INTAKE_PIVOT_CONFIG = new TalonFXConfiguration();
    static {
      ALGAE_INTAKE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      ALGAE_INTAKE_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      INTAKE_PIVOT_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      INTAKE_PIVOT_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      ALGAE_INTAKE_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      ALGAE_INTAKE_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 30;
      ALGAE_INTAKE_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;
      ALGAE_INTAKE_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 0.5;

      INTAKE_PIVOT_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      INTAKE_PIVOT_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_POS.in(Units.Rotations);
      INTAKE_PIVOT_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      INTAKE_PIVOT_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_POS.in(Units.Rotations);

      // Why don't scientists trust atoms? Because they make up everything!
      // Why do crabs never share their things? - Because they are shellfish!

      INTAKE_PIVOT_CONFIG.Feedback.SensorToMechanismRatio = 1000 / 27;

      INTAKE_PIVOT_CONFIG.Slot0.kG = 0.53; // Volts to overcome gravity
      INTAKE_PIVOT_CONFIG.Slot0.kS = 0.5; // Volts to overcome static friction
      INTAKE_PIVOT_CONFIG.Slot0.kV = 0.0; // Volts for a velocity target of 1 rps
      INTAKE_PIVOT_CONFIG.Slot0.kA = 0.0; // Volts for an acceleration of 1 rps/s
      INTAKE_PIVOT_CONFIG.Slot0.kP = 25;
      INTAKE_PIVOT_CONFIG.Slot0.kI = 0.0;
      INTAKE_PIVOT_CONFIG.Slot0.kD = 0.00;

      INTAKE_PIVOT_CONFIG.Slot1.kG = 0.5; // Volts to overcome gravity
      INTAKE_PIVOT_CONFIG.Slot1.kS = 0.5; // Volts to overcome static friction
      INTAKE_PIVOT_CONFIG.Slot1.kV = 0.0; // Volts for a velocity target of 1 rps
      INTAKE_PIVOT_CONFIG.Slot1.kA = 0.0; // Volts for an acceleration of 1 rps/s
      INTAKE_PIVOT_CONFIG.Slot1.kP = 25;
      INTAKE_PIVOT_CONFIG.Slot1.kI = 0.0;
      INTAKE_PIVOT_CONFIG.Slot1.kD = 0.00;

      INTAKE_PIVOT_CONFIG.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
      INTAKE_PIVOT_CONFIG.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

      INTAKE_PIVOT_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 9999;
      INTAKE_PIVOT_CONFIG.MotionMagic.MotionMagicAcceleration = 9999;

      INTAKE_PIVOT_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      INTAKE_PIVOT_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 30;
      INTAKE_PIVOT_CONFIG.CurrentLimits.SupplyCurrentLimit = 45;
      INTAKE_PIVOT_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 0.5;
    }

    public static final Distance REQUIRED_CORAL_DISTANCE = Units.Meters.of(0.1);
    public static TalonFXConfiguration CORAL_OUTTAKE_CONFIG = new TalonFXConfiguration();
    public static CANrangeConfiguration CORAL_SENSOR_CONFIG = new CANrangeConfiguration();
    static {
      CORAL_OUTTAKE_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      CORAL_OUTTAKE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      CORAL_SENSOR_CONFIG.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
      CORAL_SENSOR_CONFIG.ProximityParams.ProximityThreshold = REQUIRED_CORAL_DISTANCE.in(Units.Meters);

      CORAL_OUTTAKE_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      CORAL_OUTTAKE_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 30;
      CORAL_OUTTAKE_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;
      CORAL_OUTTAKE_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 0.5;
    }

    public static final Angle DEADZONE_DISTANCE = Units.Degrees.of(4);
    public static final Angle PREP_NET_PIVOT_POSITION = Units.Degrees.of(4);
    public static final Angle PREP_PROCESSOR_PIVOT_POSITION = Units.Degrees.of(4);
  }

  public static class constClimber {
    public static final TalonFXConfiguration CAGE_COLLECTOR_CONFIG = new TalonFXConfiguration();
    static {
      CAGE_COLLECTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      CAGE_COLLECTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
  }

  public static class constVision {
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

  }
}
