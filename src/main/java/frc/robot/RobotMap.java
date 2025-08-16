package frc.robot;

// Contains all ports on our robot

public class RobotMap {
  public static class mapControllers {
    public static final int DRIVER_USB = 0;
    public static final int OPERATOR_USB = 1;
  }

  public static class mapDrivetrain {
    public static final String CAN_BUS_NAME = "Swerve";
    public static final int PIGEON_CAN = 0;

    // Module 0
    public static final int FRONT_LEFT_DRIVE_CAN = 0;
    public static final int FRONT_LEFT_STEER_CAN = 1;
    public static final int FRONT_LEFT_ABSOLUTE_ENCODER_CAN = 0;

    // Module 1
    public static final int FRONT_RIGHT_DRIVE_CAN = 2;
    public static final int FRONT_RIGHT_STEER_CAN = 3;
    public static final int FRONT_RIGHT_ABSOLUTE_ENCODER_CAN = 1;

    // Module 2
    public static final int BACK_LEFT_DRIVE_CAN = 4;
    public static final int BACK_LEFT_STEER_CAN = 5;
    public static final int BACK_LEFT_ABSOLUTE_ENCODER_CAN = 2;

    // Module 3
    public static final int BACK_RIGHT_DRIVE_CAN = 6;
    public static final int BACK_RIGHT_STEER_CAN = 7;
    public static final int BACK_RIGHT_ABSOLUTE_ENCODER_CAN = 3;
  }

  // Rotors ID 11-20
  public static class mapRotors {
    // coral intake motors and pivot
    public static final int CORAL_INTAKE_LEFT_CAN = 11;

    public static final int CORAL_INTAKE_RIGHT_CAN = 10;
    // Algae intake motor
    public static final int INTAKE_ALGAE_CAN = 12;

    // Coral intake sensors
    public static final int CORAL_MID_SENSOR = 13;
    public static final int CORAL_LEFT_SENSOR = 15;
    public static final int CORAL_RIGHT_SENSOR = 16;

    public static final int CAGE_COLLECTER_CAN = 14;
  }

  // Motion ID 21-30
  public static class mapMotion {
    public static final int LEFT_LIFT_CAN = 21;
    public static final int RIGHT_LIFT_CAN = 22;
    public static final int RIGHT_PIVOT_CAN = 23;
    public static final int LEFT_PIVOT_CAN = 24;
    public static final int INTAKE_PIVOT_CAN = 25;
  }
}
