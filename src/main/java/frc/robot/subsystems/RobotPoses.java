// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.constRobotPoses;

@Logged
public class RobotPoses extends SubsystemBase {
  /** Creates a new RobotPoses. */
  MechanismLigament2d elevatorPivot; // Pivot point for the elevator
  MechanismLigament2d elevator;
  MechanismLigament2d intakeWrist;
  Mechanism2d mech = new Mechanism2d(3, 3);

  @NotLogged
  Drivetrain subDrivetrain;
  Elevator subElevator;
  Intake subIntake;

  Pose3d comp0Drivetrain = Pose3d.kZero;
  Pose3d comp1Bumpers = Pose3d.kZero.plus(Constants.ROBOT_TO_BUMPERS);

  public RobotPoses(Drivetrain subDrivetrain, Elevator subElevator, Intake subIntake) {
    this.subDrivetrain = subDrivetrain;
    this.subElevator = subElevator;
    this.subIntake = subIntake;

    // the main mechanism object

    // the mechanism root node
    MechanismRoot2d root = mech.getRoot("drivetrain", constRobotPoses.ROOT_X, constRobotPoses.ROOT_Y);

    elevatorPivot = root.append(new MechanismLigament2d("elevator-pivot", constRobotPoses.ELEVATOR_PIVOT_LENGTH,
        constRobotPoses.ELEVATOR_PIVOT_DEFAULT_ANGLE, constRobotPoses.ELEVATOR_PIVOT_WIDTH,
        new Color8Bit(Color.kGreen)));
    elevator = elevatorPivot.append(
        new MechanismLigament2d("elevator", constRobotPoses.ELEVATOR_LENGTH, constRobotPoses.ELEVATOR_DEFAULT_ANGLE,
            constRobotPoses.ELEVATOR_WIDTH, new Color8Bit(Color.kBlue)));
    // Set default motor configurations if needed
    // e.g., elevatorLeftMotor.configFactoryDefault();
    // post the mechanism to the dashboard

    intakeWrist = elevator.append(
        new MechanismLigament2d("intake-wrist", constRobotPoses.INTAKE_WRIST_LENGTH,
            constRobotPoses.INTAKE_WRIST_DEFAULT_ANGLE, constRobotPoses.INTAKE_WRIST_WIDTH, new Color8Bit(Color.kRed)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevator.setLength(subElevator.getLiftPosition().magnitude());
    elevatorPivot.setAngle(subElevator.getPivotAngle().magnitude());

    SmartDashboard.putData("Mech2d", mech);
    // Robot Positions

    comp0Drivetrain = new Pose3d(subDrivetrain.getPose());
  }
}
