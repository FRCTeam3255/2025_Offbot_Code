package frc.robot;

// Contains all ports on our robot

public class RobotMap {
  public static class mapControllers {
    public static final int DRIVER_USB = 0;
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

  // Intake ID 11-20
  public static class mapIntake {
    // coral intake motors and pivot
    public static final int INTAKE_PIVOT_CAN = 11;
    public static final int CORAL_LEFT_CAN = 12;
    public static final int CORAL_RIGHT_CAN = 13;
    // Algae intake motor
    public static final int INTAKE_ALGAE_CAN = 14;
  }

  // Elevator ID 21-30
  public static class mapElevator {
    public static final int ELEVATOR_LEFT_CAN = 21;
    public static final int ELEVATOR_RIGHT_CAN = 22;
    public static final int ELEVATOR_RIGHT_PIVOT_CAN = 23;
    public static final int ELEVATOR_LEFT_PIVOT_CAN = 24;
  }

  // Climber ID 31-40
  public static class mapClimber {
    public static final int CLIMBER_LEFT_CAN = 31;
  }
}
