// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.constField;
import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private boolean bothSubsystemsZeroed = false;

  // Why don't skeletons fight each other? They don't have the guts.
  // What do you call fake spaghetti? An impasta.
  // Why did the scarecrow win an award? Because he was outstanding in his field.
  @Override
  public void robotInit() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    Epilogue.bind(this);
    m_robotContainer = new RobotContainer();
    // Set out log file to be in its own folder
    if (Robot.isSimulation()) {
      DataLogManager.start("logs");
    } else {
      DataLogManager.start();
    }
    // Log data that is being put to shuffleboard
    DataLogManager.logNetworkTables(true);
    // Log the DS data and joysticks
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    DriverStation.silenceJoystickConnectionWarning(Constants.constControllers.SILENCE_JOYSTICK_WARNINGS);
    m_robotContainer.manualZeroLift.schedule();
    m_robotContainer.manualZeroPivot.schedule();
    m_robotContainer.manualZeroWrist.schedule();
    m_robotContainer.startingCofig.schedule();
  }

  // simulation period method in your Robot.java
  // DO NOT call SimulatedArena.getInstance().simulationPeriodic() when running on
  // a REAL robot, as it will drain the resources of your roboRIO.
  @Override
  public void simulationPeriodic() {
    if (Robot.isSimulation()) {
      SimulatedArena.getInstance().simulationPeriodic();

      // Add a Crescendo note to the field
      SimulatedArena.getInstance().addGamePiece(new CrescendoNoteOnField(new Translation2d(3, 3)));

      // Clear all game pieces from the field
      SimulatedArena.getInstance().clearGamePieces();

      // Create and configure a drivetrain simulation configuration
      final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
          // Specify gyro type (for realistic gyro drifting and error simulation)
          .withGyro(COTS.ofPigeon2())
          // Specify swerve module (for realistic swerve dynamics)
          .withSwerveModule(new SwerveModuleSimulationConfig(
              DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
              DCMotor.getFalcon500(1), // Steer motor is a Falcon 500
              6.12, // Drive motor gear ratio.
              12.8, // Steer motor gear ratio.
              Volts.of(0.1), // Drive friction voltage.
              Volts.of(0.1), // Steer friction voltage
              Inches.of(2), // Wheel radius
              KilogramSquareMeters.of(0.03), // Steer MOI
              1.2)) // Wheel COF
          // Configures the track length and track width (spacing between swerve modules)
          .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
          // Configures the bumper size (dimensions of the robot bumper)
          .withBumperSize(Inches.of(30), Inches.of(30));
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.AddVisionMeasurement().schedule();

  }

  @Override
  public void disabledInit() {
    Elastic.selectTab("Disabled");
    bothSubsystemsZeroed = m_robotContainer.allZeroed();
  }

  @Override
  public void disabledPeriodic() {
    constField.ALLIANCE = DriverStation.getAlliance();
    SmartDashboard.putString("ALLIANCE", constField.ALLIANCE.toString());
  }

  @Override
  public void disabledExit() {
    m_robotContainer.manualZeroLift.cancel();
    m_robotContainer.manualZeroPivot.cancel();
    m_robotContainer.manualZeroWrist.cancel();
    m_robotContainer.startingCofig.cancel();
  }

  @Override
  public void autonomousInit() {
    Elastic.selectTab("Autonomous");

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    bothSubsystemsZeroed = m_robotContainer.allZeroed();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    m_robotContainer.manualZeroLift.cancel();
    m_robotContainer.manualZeroPivot.cancel();
    m_robotContainer.manualZeroWrist.cancel();
    m_robotContainer.startingCofig.cancel();
  }

  @Override
  public void teleopInit() {
    m_robotContainer.manualZeroLift.cancel();
    m_robotContainer.manualZeroPivot.cancel();
    m_robotContainer.manualZeroWrist.cancel();
    m_robotContainer.startingCofig.cancel();

    Elastic.selectTab("Teleoperated");

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
