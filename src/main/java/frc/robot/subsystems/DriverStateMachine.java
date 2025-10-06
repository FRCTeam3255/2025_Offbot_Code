// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constPoseDrive;
import frc.robot.commands.driver_states.DriveManual;
import frc.robot.commands.driver_states.PoseDriving;

@Logged
public class DriverStateMachine extends SubsystemBase {
  /** Creates a new DriverStateMachine. */
  public static DriverState currentDriverState = DriverState.MANUAL;

  @NotLogged
  Drivetrain subDrivetrain;
  @NotLogged
  DriverStateMachine subDriverStateMachine = this;

  public DriverStateMachine(Drivetrain subDrivetrain) {
    currentDriverState = DriverState.MANUAL;
    this.subDrivetrain = subDrivetrain;
  }

  public DriverState getDriverState() {
    return currentDriverState;
  }

  public void setDriverState(DriverState driverState) {
    currentDriverState = driverState;
  }

  public enum DriverState {
    MANUAL,
    REEF_ROTATION_SNAPPING,
    CORAL_STATION_ROTATION_SNAPPING,
    REEF_AUTO_DRIVING_LEFT,
    REEF_AUTO_DRIVING_RIGHT,
    CORAL_STATION_AUTO_DRIVING_FAR,
    CORAL_STATION_AUTO_DRIVING_CLOSE,
    PROCESSOR_ROTATION_SNAPPING,
    PROCESSOR_AUTO_DRIVING,
    NET_ROTATION_SNAPPING,
    NET_AUTO_DRIVING,
    ALGAE_ROTATION_SNAPPING,
    ALGAE_AUTO_DRIVING,
    CAGE_ROTATION_SNAPPING
    // TODO: Add other driver states as needed
  }

  public Command tryState(DriverState desiredState, DoubleSupplier xAxis,
      DoubleSupplier yAxis, DoubleSupplier rotationAxis, BooleanSupplier slowMode) {
    switch (desiredState) {
      case MANUAL:
        switch (currentDriverState) {
          case MANUAL:
          case REEF_ROTATION_SNAPPING:
          case CORAL_STATION_ROTATION_SNAPPING:
          case REEF_AUTO_DRIVING_LEFT:
          case REEF_AUTO_DRIVING_RIGHT:
          case CORAL_STATION_AUTO_DRIVING_FAR:
          case CORAL_STATION_AUTO_DRIVING_CLOSE:
          case PROCESSOR_ROTATION_SNAPPING:
          case PROCESSOR_AUTO_DRIVING:
          case NET_ROTATION_SNAPPING:
          case NET_AUTO_DRIVING:
          case ALGAE_ROTATION_SNAPPING:
          case ALGAE_AUTO_DRIVING:
          case CAGE_ROTATION_SNAPPING:
            return new DriveManual(subDrivetrain, subDriverStateMachine, xAxis, yAxis, rotationAxis, slowMode);
        }

        break;

      case REEF_AUTO_DRIVING_LEFT:
        switch (currentDriverState) {
          case MANUAL:
          case REEF_ROTATION_SNAPPING:
          case CORAL_STATION_ROTATION_SNAPPING:
          case REEF_AUTO_DRIVING_RIGHT:
          case CORAL_STATION_AUTO_DRIVING_FAR:
          case CORAL_STATION_AUTO_DRIVING_CLOSE:
          case PROCESSOR_ROTATION_SNAPPING:
          case PROCESSOR_AUTO_DRIVING:
          case NET_ROTATION_SNAPPING:
          case NET_AUTO_DRIVING:
          case ALGAE_ROTATION_SNAPPING:
          case ALGAE_AUTO_DRIVING:
          case CAGE_ROTATION_SNAPPING:
            return new PoseDriving(subDrivetrain, subDriverStateMachine, xAxis, yAxis, rotationAxis,
                slowMode, constPoseDrive.CORAL_REEF_LEFT);
        }
        break;

      case REEF_AUTO_DRIVING_RIGHT:
        switch (currentDriverState) {
          case MANUAL:
          case REEF_ROTATION_SNAPPING:
          case CORAL_STATION_ROTATION_SNAPPING:
          case REEF_AUTO_DRIVING_LEFT:
          case CORAL_STATION_AUTO_DRIVING_FAR:
          case CORAL_STATION_AUTO_DRIVING_CLOSE:
          case PROCESSOR_ROTATION_SNAPPING:
          case PROCESSOR_AUTO_DRIVING:
          case NET_ROTATION_SNAPPING:
          case NET_AUTO_DRIVING:
          case ALGAE_ROTATION_SNAPPING:
          case ALGAE_AUTO_DRIVING:
          case CAGE_ROTATION_SNAPPING:
            return new PoseDriving(subDrivetrain, subDriverStateMachine, xAxis, yAxis, rotationAxis,
            slowMode, constPoseDrive.CORAL_REEF_RIGHT);
        }
        break;

      case CORAL_STATION_AUTO_DRIVING_FAR:
        switch (currentDriverState) {
          case MANUAL:
          case REEF_ROTATION_SNAPPING:
          case CORAL_STATION_ROTATION_SNAPPING:
          case REEF_AUTO_DRIVING_LEFT:
          case REEF_AUTO_DRIVING_RIGHT:
          case CORAL_STATION_AUTO_DRIVING_FAR:
          case CORAL_STATION_AUTO_DRIVING_CLOSE:
          case PROCESSOR_ROTATION_SNAPPING:
          case PROCESSOR_AUTO_DRIVING:
          case NET_ROTATION_SNAPPING:
          case NET_AUTO_DRIVING:
          case ALGAE_ROTATION_SNAPPING:
          case ALGAE_AUTO_DRIVING:
          case CAGE_ROTATION_SNAPPING:
            return new PoseDriving(subDrivetrain, subDriverStateMachine, xAxis, yAxis, rotationAxis,
            slowMode, constPoseDrive.CORAL_STATION_FAR);
        }
        break;

      case CORAL_STATION_AUTO_DRIVING_CLOSE:
        switch (currentDriverState) {
          case MANUAL:
          case REEF_ROTATION_SNAPPING:
          case CORAL_STATION_ROTATION_SNAPPING:
          case REEF_AUTO_DRIVING_LEFT:
          case REEF_AUTO_DRIVING_RIGHT:
          case CORAL_STATION_AUTO_DRIVING_FAR:
          case CORAL_STATION_AUTO_DRIVING_CLOSE:
          case PROCESSOR_ROTATION_SNAPPING:
          case PROCESSOR_AUTO_DRIVING:
          case NET_ROTATION_SNAPPING:
          case NET_AUTO_DRIVING:
          case ALGAE_ROTATION_SNAPPING:
          case ALGAE_AUTO_DRIVING:
          case CAGE_ROTATION_SNAPPING:
            return new PoseDriving(subDrivetrain, subDriverStateMachine, xAxis, yAxis, rotationAxis,
            slowMode, constPoseDrive.CORAL_STATION_CLOSE);
        }
        break;

      case PROCESSOR_ROTATION_SNAPPING:
        switch (currentDriverState) {
          case MANUAL:
          case REEF_ROTATION_SNAPPING:
          case CORAL_STATION_ROTATION_SNAPPING:
          case REEF_AUTO_DRIVING_LEFT:
          case REEF_AUTO_DRIVING_RIGHT:
          case CORAL_STATION_AUTO_DRIVING_FAR:
          case CORAL_STATION_AUTO_DRIVING_CLOSE:
          case PROCESSOR_ROTATION_SNAPPING:
          case PROCESSOR_AUTO_DRIVING:
          case NET_ROTATION_SNAPPING:
          case NET_AUTO_DRIVING:
          case ALGAE_ROTATION_SNAPPING:
          case ALGAE_AUTO_DRIVING:
          case CAGE_ROTATION_SNAPPING:
            return new PoseDriving(subDrivetrain, subDriverStateMachine, xAxis, yAxis, rotationAxis,
            slowMode, constPoseDrive.PROCESSOR);
        }
        break;

      case PROCESSOR_AUTO_DRIVING:
        switch (currentDriverState) {
          case MANUAL:
          case REEF_ROTATION_SNAPPING:
          case CORAL_STATION_ROTATION_SNAPPING:
          case REEF_AUTO_DRIVING_LEFT:
          case REEF_AUTO_DRIVING_RIGHT:
          case CORAL_STATION_AUTO_DRIVING_FAR:
          case CORAL_STATION_AUTO_DRIVING_CLOSE:
          case PROCESSOR_ROTATION_SNAPPING:
          case PROCESSOR_AUTO_DRIVING:
          case NET_ROTATION_SNAPPING:
          case NET_AUTO_DRIVING:
          case ALGAE_ROTATION_SNAPPING:
          case ALGAE_AUTO_DRIVING:
          case CAGE_ROTATION_SNAPPING:
            return new PoseDriving(subDrivetrain, subDriverStateMachine, xAxis, yAxis, rotationAxis,
            slowMode, constPoseDrive.PROCESSOR);
        }
        break;

      case NET_ROTATION_SNAPPING:
        switch (currentDriverState) {
          case MANUAL:
          case REEF_ROTATION_SNAPPING:
          case CORAL_STATION_ROTATION_SNAPPING:
          case REEF_AUTO_DRIVING_LEFT:
          case REEF_AUTO_DRIVING_RIGHT:
          case CORAL_STATION_AUTO_DRIVING_FAR:
          case CORAL_STATION_AUTO_DRIVING_CLOSE:
          case PROCESSOR_ROTATION_SNAPPING:
          case PROCESSOR_AUTO_DRIVING:
          case NET_ROTATION_SNAPPING:
          case NET_AUTO_DRIVING:
          case ALGAE_ROTATION_SNAPPING:
          case ALGAE_AUTO_DRIVING:
          case CAGE_ROTATION_SNAPPING:
            return new PoseDriving(subDrivetrain, subDriverStateMachine, xAxis, yAxis, rotationAxis,
            slowMode, constPoseDrive.NET);
        }
        break;
      case NET_AUTO_DRIVING:
        switch (currentDriverState) {
          case MANUAL:
          case REEF_ROTATION_SNAPPING:
          case CORAL_STATION_ROTATION_SNAPPING:
          case REEF_AUTO_DRIVING_LEFT:
          case REEF_AUTO_DRIVING_RIGHT:
          case CORAL_STATION_AUTO_DRIVING_FAR:
          case CORAL_STATION_AUTO_DRIVING_CLOSE:
          case PROCESSOR_ROTATION_SNAPPING:
          case PROCESSOR_AUTO_DRIVING:
          case NET_ROTATION_SNAPPING:
          case NET_AUTO_DRIVING:
          case ALGAE_ROTATION_SNAPPING:
          case ALGAE_AUTO_DRIVING:
          case CAGE_ROTATION_SNAPPING:
            return new PoseDriving(subDrivetrain, subDriverStateMachine, xAxis, yAxis, rotationAxis,
            slowMode, constPoseDrive.NET);
        }
        break;
      case ALGAE_ROTATION_SNAPPING:
        switch (currentDriverState) {
          case MANUAL:
          case REEF_ROTATION_SNAPPING:
          case CORAL_STATION_ROTATION_SNAPPING:
          case REEF_AUTO_DRIVING_LEFT:
          case REEF_AUTO_DRIVING_RIGHT:
          case CORAL_STATION_AUTO_DRIVING_FAR:
          case CORAL_STATION_AUTO_DRIVING_CLOSE:
          case PROCESSOR_ROTATION_SNAPPING:
          case PROCESSOR_AUTO_DRIVING:
          case NET_ROTATION_SNAPPING:
          case NET_AUTO_DRIVING:
          case ALGAE_ROTATION_SNAPPING:
          case ALGAE_AUTO_DRIVING:
          case CAGE_ROTATION_SNAPPING:
            return new PoseDriving(subDrivetrain, subDriverStateMachine, xAxis, yAxis, rotationAxis,
            slowMode, constPoseDrive.ALGAE_REEF);
        }
        break;
      case ALGAE_AUTO_DRIVING:
        switch (currentDriverState) {
          case MANUAL:
          case REEF_ROTATION_SNAPPING:
          case CORAL_STATION_ROTATION_SNAPPING:
          case REEF_AUTO_DRIVING_LEFT:
          case REEF_AUTO_DRIVING_RIGHT:
          case CORAL_STATION_AUTO_DRIVING_FAR:
          case CORAL_STATION_AUTO_DRIVING_CLOSE:
          case PROCESSOR_ROTATION_SNAPPING:
          case PROCESSOR_AUTO_DRIVING:
          case NET_ROTATION_SNAPPING:
          case NET_AUTO_DRIVING:
          case ALGAE_ROTATION_SNAPPING:
          case ALGAE_AUTO_DRIVING:
          case CAGE_ROTATION_SNAPPING:
            return new PoseDriving(subDrivetrain, subDriverStateMachine, xAxis, yAxis, rotationAxis,
            slowMode, constPoseDrive.ALGAE_REEF);
        }
        break;
      case CAGE_ROTATION_SNAPPING:
        switch (currentDriverState) {
          case MANUAL:
          case REEF_ROTATION_SNAPPING:
          case CORAL_STATION_ROTATION_SNAPPING:
          case REEF_AUTO_DRIVING_LEFT:
          case REEF_AUTO_DRIVING_RIGHT:
          case CORAL_STATION_AUTO_DRIVING_FAR:
          case CORAL_STATION_AUTO_DRIVING_CLOSE:
          case PROCESSOR_ROTATION_SNAPPING:
          case PROCESSOR_AUTO_DRIVING:
          case NET_ROTATION_SNAPPING:
          case NET_AUTO_DRIVING:
          case ALGAE_ROTATION_SNAPPING:
          case ALGAE_AUTO_DRIVING:
          case CAGE_ROTATION_SNAPPING:
            return new PoseDriving(subDrivetrain, subDriverStateMachine, xAxis, yAxis, rotationAxis,
            slowMode, constPoseDrive.CAGE);
        }
        break;

    }
    return Commands.print("ITS SO OVER D: Invalid Driver State Provided, Blame Eli. Attempted to go to: "
        + desiredState.toString() + " while at " + currentDriverState.toString());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}