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
import com.frcteam3255.joystick.SN_XboxController;

@Logged
public class RobotPoses extends SubsystemBase {
  /** Creates a new RobotPoses. */
  MechanismLigament2d elevator;
  private MechanismLigament2d wrist;
  Mechanism2d mech = new Mechanism2d(3, 3);

  @NotLogged
  Drivetrain subDrivetrain;
  Elevator subElevator;

  Pose3d comp0Drivetrain = Pose3d.kZero;
  Pose3d comp1Bumpers = Pose3d.kZero.plus(Constants.ROBOT_TO_BUMPERS);

  public RobotPoses(Drivetrain subDrivetrain, Elevator subElevator) {
    this.subDrivetrain = subDrivetrain;
    this.subElevator = subElevator;

    // the main mechanism object

    // the mechanism root node
    MechanismRoot2d root = mech.getRoot("climber", 1.5, .6);
    elevator = root.append(new MechanismLigament2d("elevator", .5, 90));
    // Set default motor configurations if needed
    // e.g., elevatorLeftMotor.configFactoryDefault();
    // post the mechanism to the dashboard
    wrist = elevator.append(
        new MechanismLigament2d("wrist", 0.5, 95, 10, new Color8Bit(Color.kPurple)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevator.setLength(2);
    elevator.setAngle(subElevator.getAngle().magnitude());

    SmartDashboard.putData("Mech2d", mech);
    // Robot Positions

    comp0Drivetrain = new Pose3d(subDrivetrain.getPose());
  }
}
