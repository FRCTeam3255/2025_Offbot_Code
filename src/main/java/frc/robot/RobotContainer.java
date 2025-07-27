// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.constControllers;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.StateMachine.RobotState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;

@Logged
public class RobotContainer {

  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);

  private final Drivetrain subDrivetrain = new Drivetrain();
  private final StateMachine subStateMachine = new StateMachine(subDrivetrain);
  private final Elevator subElevator = new Elevator();
  private final RobotPoses robotPose = new RobotPoses(subDrivetrain, subElevator);

  Command TRY_NONE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.NONE));

  public RobotContainer() {
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    subDrivetrain
        .setDefaultCommand(
            new DriveManual(subDrivetrain, subStateMachine, conDriver.axis_LeftY, conDriver.axis_LeftX,
                conDriver.axis_RightX));

    configDriverBindings();
    configOperatorBindings();

    subDrivetrain.resetModulesToAbsolute();
  }

  private void configDriverBindings() {
    conDriver.btn_A.onTrue(Commands.runOnce(() -> subElevator.setAngle(Degrees.of(1000))));
    conDriver.btn_Y.onTrue(Commands.runOnce(() -> subElevator.setAngle(Degrees.of(-5000))));
    conDriver.btn_B.onTrue(Commands.runOnce(() -> subDrivetrain.resetModulesToAbsolute()));
    conDriver.btn_Back
        .onTrue(Commands.runOnce(() -> subDrivetrain.resetPoseToPose(new Pose2d(0, 0, new Rotation2d()))));

    // Defaults to Field-Relative, is Robot-Relative while held
    conDriver.btn_LeftBumper
        .whileTrue(Commands.runOnce(() -> subDrivetrain.setRobotRelative()))
        .onFalse(Commands.runOnce(() -> subDrivetrain.setFieldRelative()));
  }

  public Command getAutonomousCommand() {
    return Commands.runOnce(() -> subDrivetrain.resetPoseToPose(Constants.constField.WORKSHOP_STARTING_POSE))
        .andThen(new ExampleAuto(subDrivetrain));
  }

  private void configOperatorBindings() {
    // Add operator bindings here if needed
  }

  public RobotState getRobotState() {
    return subStateMachine.getRobotState();
  }
}
