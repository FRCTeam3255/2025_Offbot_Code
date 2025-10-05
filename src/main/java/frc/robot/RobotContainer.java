// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;
import java.util.Set;

import com.frcteam3255.joystick.SN_XboxController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.constControllers;
import frc.robot.Constants.constField;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.*;
import frc.robot.commands.driver_states.DriveManual;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DriverStateMachine.DriverState;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.commands.Zeroing.*;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;

@Logged
public class RobotContainer {
  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  private final SN_XboxController conOperator = new SN_XboxController(mapControllers.OPERATOR_USB);

  private final Drivetrain subDrivetrain = new Drivetrain();
  private final Rotors subRotors = new Rotors();
  private final Motion subMotion = new Motion();
  private final LED subLED = new LED();
  private final StateMachine subStateMachine = new StateMachine(subDrivetrain, subRotors, subMotion);
  private final DriverStateMachine subDriverStateMachine = new DriverStateMachine(subDrivetrain);
  private final RobotPoses robotPose = new RobotPoses(subDrivetrain, subMotion, subRotors);
  private final Vision subVision = new Vision();

  public Command manualZeroLift = new ManualZeroLift(subMotion, subLED).ignoringDisable(true);
  public Command manualZeroPivot = new ManualZeroPivot(subMotion, subLED).ignoringDisable(true);
  public Command manualZeroWrist = new ManualZeroWrist(subMotion, subLED).ignoringDisable(true);
  public Command startingCofig = new StartingConfig(subMotion, subLED).ignoringDisable(true);

  private final Trigger hasCoralTrigger = new Trigger(() -> subRotors.hasCoral() && !subRotors.hasAlgae());
  private final Trigger hasAlgaeTrigger = new Trigger(() -> !subRotors.hasCoral() && subRotors.hasAlgae());
  private final Trigger hasBothTrigger = new Trigger(() -> subRotors.hasCoral() && subRotors.hasAlgae());
  private final Trigger isInCleaningStates = new Trigger(() -> subStateMachine.inCleaningState());
  private final Trigger hasCoralL1Trigger = new Trigger(() -> subRotors.hasL1Coral());
  private final Trigger isCageLatchedTrigger = new Trigger(() -> subRotors.isCageLatched());
  private final Trigger isInCSAutoDriveState = new Trigger(
      () -> subDriverStateMachine.getDriverState() == DriverStateMachine.DriverState.CORAL_STATION_AUTO_DRIVING_FAR
          || subDriverStateMachine.getDriverState() == DriverStateMachine.DriverState.CORAL_STATION_AUTO_DRIVING_CLOSE);
  private final Trigger isInProcessorAutoDriveState = new Trigger(
      () -> subDriverStateMachine.getDriverState() == DriverStateMachine.DriverState.PROCESSOR_AUTO_DRIVING);

  Command TRY_NONE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.NONE));
  Command TRY_CLIMBING = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.CLIMBING));
  Command TRY_PREP_CLIMB = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CLIMB));
  Command TRY_PREP_CORAL_ZERO = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_ZERO));
  Command TRY_PREP_CORAL_L1 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_L1));
  Command TRY_PREP_CORAL_L2 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_L2));
  Command TRY_PREP_CORAL_L3 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_L3));
  Command TRY_PREP_CORAL_L4 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_L4));
  Command TRY_PREP_CORAL_L2_WITH_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_L2_WITH_ALGAE));
  Command TRY_PREP_CORAL_L3_WITH_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_L3_WITH_ALGAE));
  Command TRY_PREP_CORAL_L4_WITH_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_L4_WITH_ALGAE));
  Command TRY_PREP_CORAL_ZERO_WITH_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_CORAL_ZERO_WITH_ALGAE));
  Command TRY_PREP_ALGAE_NET = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_ALGAE_NET));
  Command TRY_PREP_ALGAE_PROCESSOR = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_ALGAE_PROCESSOR));
  Command TRY_PREP_ALGAE_ZERO = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_ALGAE_ZERO));
  Command TRY_PREP_ALGAE_NET_WITH_CORAL = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_ALGAE_NET_WITH_CORAL));
  Command TRY_PREP_ALGAE_PROCESSOR_WITH_CORAL = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.PREP_ALGAE_PROCESSOR_WITH_CORAL));
  Command TRY_HAS_CORAL = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.HAS_CORAL));
  Command TRY_HAS_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.HAS_ALGAE));
  Command TRY_HAS_CORAL_AND_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.HAS_CORAL_AND_ALGAE));
  Command TRY_SCORING_CORAL = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.SCORING_CORAL));
  Command TRY_SCORING_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.SCORING_ALGAE));
  Command TRY_SCORING_CORAL_L1 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.SCORING_CORAL_L1));
  Command TRY_CLEAN_HIGH = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.CLEAN_HIGH));
  Command TRY_CLEAN_LOW = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.CLEAN_LOW));
  Command TRY_INTAKE_CORAL_STATION = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.INTAKE_CORAL_STATION));
  Command TRY_INTAKE_ALGAE_GROUND = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.INTAKE_ALGAE_GROUND));
  Command TRY_EJECTING = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.EJECTING));
  Command TRY_SCORING_ALGAE_WITH_CORAL = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.SCORING_ALGAE_WITH_CORAL));
  Command TRY_SCORING_CORAL_WITH_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.SCORING_CORAL_WITH_ALGAE));
  Command TRY_CLEAN_HIGH_WITH_CORAL = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.CLEAN_HIGH_WITH_CORAL));
  Command TRY_CLEAN_LOW_WITH_CORAL = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.CLEAN_LOW_WITH_CORAL));
  Command TRY_INTAKE_CORAL_GROUND = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.INTAKE_CORAL_GROUND));
  Command TRY_INTAKE_CORAL_GROUND_WITH_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.INTAKE_CORAL_GROUND_WITH_ALGAE));
  Command TRY_INTAKE_ALGAE_GROUND_WITH_CORAL = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.INTAKE_ALGAE_GROUND_WITH_CORAL));
  Command TRY_INTAKE_CORAL_STATION_WITH_ALGAE = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.INTAKE_CORAL_STATION_WITH_ALGAE));
  Command TRY_INTAKE_CORAL_L1 = Commands.deferredProxy(
      () -> subStateMachine.tryState(RobotState.INTAKE_CORAL_L1));
  Command HAS_CORAL_OVERRIDE = Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.HAS_CORAL)
      .alongWith(subStateMachine.tryState(RobotState.HAS_CORAL_AND_ALGAE)));
  Command HAS_ALGAE_OVERRIDE = Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.HAS_ALGAE)
      .alongWith(subStateMachine.tryState(RobotState.HAS_CORAL_AND_ALGAE)));
  Command HAS_CORAL_L1_OVERRIDE = Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.PREP_CORAL_L1));

  // --- Driver State Commands ---
  Command MANUAL = Commands.deferredProxy(
      () -> subDriverStateMachine.tryState(DriverStateMachine.DriverState.MANUAL, conDriver.axis_LeftY,
          conDriver.axis_LeftX, conDriver.axis_RightX));
  Command REEF_ROTATION_SNAPPING = Commands.deferredProxy(
      () -> subDriverStateMachine.tryState(DriverStateMachine.DriverState.REEF_ROTATION_SNAPPING,
          conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX));
  Command CORAL_STATION_ROTATION_SNAPPING = Commands.deferredProxy(
      () -> subDriverStateMachine.tryState(DriverStateMachine.DriverState.CORAL_STATION_ROTATION_SNAPPING,
          conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX));
  Command REEF_AUTO_DRIVING_LEFT = Commands.deferredProxy(
      () -> subDriverStateMachine.tryState(DriverStateMachine.DriverState.REEF_AUTO_DRIVING_LEFT,
          conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX));
  Command REEF_AUTO_DRIVING_RIGHT = Commands.deferredProxy(
      () -> subDriverStateMachine.tryState(DriverStateMachine.DriverState.REEF_AUTO_DRIVING_RIGHT,
          conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX));
  Command CORAL_STATION_AUTO_DRIVING_FAR = Commands.deferredProxy(
      () -> subDriverStateMachine.tryState(DriverStateMachine.DriverState.CORAL_STATION_AUTO_DRIVING_FAR,
          conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX));
  Command CORAL_STATION_AUTO_DRIVING_CLOSE = Commands.deferredProxy(
      () -> subDriverStateMachine.tryState(DriverStateMachine.DriverState.CORAL_STATION_AUTO_DRIVING_CLOSE,
          conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX));
  Command PROCESSOR_ROTATION_SNAPPING = Commands.deferredProxy(
      () -> subDriverStateMachine.tryState(DriverStateMachine.DriverState.PROCESSOR_ROTATION_SNAPPING,
          conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX));
  Command PROCESSOR_AUTO_DRIVING = Commands.deferredProxy(
      () -> subDriverStateMachine.tryState(DriverStateMachine.DriverState.PROCESSOR_AUTO_DRIVING,
          conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX));
  Command NET_ROTATION_SNAPPING = Commands.deferredProxy(
      () -> subDriverStateMachine.tryState(DriverStateMachine.DriverState.NET_ROTATION_SNAPPING,
          conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX));
  Command NET_AUTO_DRIVING = Commands.deferredProxy(
      () -> subDriverStateMachine.tryState(DriverStateMachine.DriverState.NET_AUTO_DRIVING,
          conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX));
  Command ALGAE_ROTATION_SNAPPING = Commands.deferredProxy(
      () -> subDriverStateMachine.tryState(DriverStateMachine.DriverState.ALGAE_ROTATION_SNAPPING,
          conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX));
  Command ALGAE_AUTO_DRIVING = Commands.deferredProxy(
      () -> subDriverStateMachine.tryState(DriverStateMachine.DriverState.ALGAE_AUTO_DRIVING,
          conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX));
  Command CAGE_ROTATION_SNAPPING = Commands.deferredProxy(
      () -> subDriverStateMachine.tryState(DriverStateMachine.DriverState.CAGE_ROTATION_SNAPPING,
          conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX));

  @NotLogged
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  Pair<RobotState, Pose2d>[] SELECTED_AUTO_PREP_MAP;
  public static String SELECTED_AUTO_PREP_MAP_NAME = "none :("; // For logging :p
  int AUTO_PREP_NUM = 0;

  public RobotContainer() {
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    subDrivetrain
        .setDefaultCommand(new DriveManual(
            subDrivetrain, subDriverStateMachine, conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX));

    configDriverBindings();
    configOperatorBindings();

    subDrivetrain.resetModulesToAbsolute();
  }

  private void configDriverBindings() {
    conDriver.btn_B.onTrue(Commands.runOnce(() -> subDrivetrain.resetModulesToAbsolute()));
    conDriver.btn_North
        .onTrue(Commands
            .runOnce(() -> subDrivetrain.resetPoseToPose(constField.RESET_POS)));

    conDriver.btn_LeftTrigger
        .whileTrue(REEF_AUTO_DRIVING_LEFT).and(isInCleaningStates.negate())
        .onFalse(MANUAL);

    conDriver.btn_RightTrigger.and(isInCleaningStates.negate())
        .whileTrue(REEF_AUTO_DRIVING_RIGHT)
        .onFalse(MANUAL);

    conDriver.btn_LeftTrigger.and(isInCleaningStates)
        .whileTrue(ALGAE_AUTO_DRIVING)
        .onFalse(MANUAL);

    conDriver.btn_LeftTrigger.and(isInCleaningStates)
        .whileTrue(ALGAE_ROTATION_SNAPPING)
        .onFalse(MANUAL);

    conDriver.btn_RightTrigger.and(isInCleaningStates)
        .whileTrue(ALGAE_AUTO_DRIVING)
        .onFalse(MANUAL);

    conDriver.btn_RightTrigger.and(isInCleaningStates)
        .whileTrue(ALGAE_ROTATION_SNAPPING)
        .onFalse(MANUAL);

    conDriver.btn_X
        .whileTrue(CORAL_STATION_AUTO_DRIVING_FAR)
        .onFalse(MANUAL);

    conDriver.btn_B
        .whileTrue(CORAL_STATION_AUTO_DRIVING_CLOSE)
        .onFalse(MANUAL);

    conDriver.btn_East
        .whileTrue(PROCESSOR_AUTO_DRIVING)
        .onFalse(MANUAL);

    conDriver.btn_East
        .whileTrue(PROCESSOR_ROTATION_SNAPPING)
        .onFalse(MANUAL);

    conDriver.btn_South
        .whileTrue(CAGE_ROTATION_SNAPPING)
        .onFalse(MANUAL);

    conDriver.btn_LeftBumper
        .whileTrue(NET_AUTO_DRIVING)
        .onFalse(MANUAL);

    conDriver.btn_LeftBumper
        .whileTrue(NET_ROTATION_SNAPPING)
        .onFalse(MANUAL);

    conDriver.btn_Start
        .onTrue(TRY_PREP_CLIMB);

    conDriver.btn_Y
        .whileTrue(TRY_CLIMBING);

    isInCSAutoDriveState
        .whileTrue(TRY_INTAKE_CORAL_STATION)
        .onFalse(TRY_NONE);

    isInProcessorAutoDriveState
        .whileTrue(TRY_PREP_ALGAE_PROCESSOR)
        .whileTrue(TRY_PREP_ALGAE_PROCESSOR_WITH_CORAL);
  }

  private void configOperatorBindings() {
    // Add operator bindings here if needed
    conOperator.btn_LeftTrigger

        .whileTrue(TRY_INTAKE_CORAL_GROUND)
        .whileTrue(TRY_INTAKE_CORAL_GROUND_WITH_ALGAE)
        .onFalse(TRY_NONE)
        .onFalse(TRY_HAS_ALGAE);

    conOperator.btn_LeftBumper

        .whileTrue(TRY_INTAKE_ALGAE_GROUND)
        .whileTrue(TRY_INTAKE_ALGAE_GROUND_WITH_CORAL)
        .onFalse(TRY_NONE)
        .onFalse(TRY_HAS_CORAL);

    conOperator.btn_RightTrigger

        .whileTrue(TRY_SCORING_CORAL)
        .whileTrue(TRY_SCORING_ALGAE)
        .whileTrue(TRY_SCORING_ALGAE_WITH_CORAL)
        .whileTrue(TRY_SCORING_CORAL_WITH_ALGAE)
        .whileTrue(TRY_SCORING_CORAL_L1)
        .onFalse(TRY_NONE)
        .onFalse(TRY_HAS_CORAL)
        .onFalse(TRY_HAS_ALGAE);

    conOperator.btn_RightBumper

        .whileTrue(TRY_INTAKE_CORAL_STATION)
        .whileTrue(TRY_INTAKE_CORAL_STATION_WITH_ALGAE)
        .onFalse(TRY_NONE)
        .onFalse(TRY_HAS_ALGAE);

    conOperator.btn_A
        .whileTrue(TRY_INTAKE_CORAL_L1)
        .onFalse(TRY_NONE);

    conOperator.btn_B
        .onTrue(TRY_PREP_CORAL_L3)
        .onTrue(TRY_PREP_CORAL_L3_WITH_ALGAE);

    conOperator.btn_X
        .onTrue(TRY_PREP_CORAL_L2)
        .onTrue(TRY_PREP_CORAL_L2_WITH_ALGAE);

    conOperator.btn_Y
        .onTrue(TRY_PREP_CORAL_L4)
        .onTrue(TRY_PREP_CORAL_L4_WITH_ALGAE);

    conOperator.btn_LeftStick
        .whileTrue(TRY_EJECTING)
        .onFalse(TRY_NONE);

    conOperator.btn_RightStick
        .onTrue(TRY_PREP_CORAL_ZERO)
        .onTrue(TRY_PREP_CORAL_ZERO_WITH_ALGAE)
        .onTrue(TRY_PREP_ALGAE_ZERO);

    conOperator.btn_North
        .onTrue(TRY_PREP_ALGAE_NET)
        .onTrue(TRY_PREP_ALGAE_NET_WITH_CORAL);

    conOperator.btn_South
        .onTrue(TRY_PREP_ALGAE_PROCESSOR)
        .onTrue(TRY_PREP_ALGAE_PROCESSOR_WITH_CORAL);

    conOperator.btn_East
        .whileTrue(TRY_CLEAN_HIGH)
        .whileTrue(TRY_CLEAN_HIGH_WITH_CORAL)
        .onFalse(TRY_NONE)
        .onFalse(TRY_HAS_CORAL);

    conOperator.btn_West
        .whileTrue(TRY_CLEAN_LOW)
        .whileTrue(TRY_CLEAN_LOW_WITH_CORAL)
        .onFalse(TRY_NONE)
        .onFalse(TRY_HAS_CORAL);

    conOperator.btn_Start
        .onTrue(HAS_CORAL_OVERRIDE)
        .onTrue(HAS_CORAL_L1_OVERRIDE);

    conOperator.btn_Back.onTrue(HAS_ALGAE_OVERRIDE);

    hasCoralTrigger.debounce(0.1)
        .whileTrue(TRY_HAS_CORAL);

    hasAlgaeTrigger// debounce(0.2).and(conOperator.btn_West.negate()).and(conOperator.btn_East.negate())
        .whileTrue(TRY_HAS_ALGAE);

    hasBothTrigger
        .whileTrue(TRY_HAS_CORAL_AND_ALGAE);

    hasCoralL1Trigger.debounce(0.1)
        .whileTrue(TRY_PREP_CORAL_L1);

    isCageLatchedTrigger
        .onTrue(TRY_CLIMBING);
  }

  public boolean allZeroed() {
    return subMotion.hasLiftZeroed && subMotion.hasPivotZeroed && subMotion.hasWristZeroed;
  }

  public RobotState getRobotState() {
    return subStateMachine.getRobotState();
  }

  public Command AddVisionMeasurement() {
    return new AddVisionMeasurement(subDrivetrain, subVision)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).ignoringDisable(true);
  }

    // ------ Autos ------
  public Command getAutonomousCommand() {
    return Commands.runOnce(() -> subDrivetrain.resetPoseToPose(Constants.constField.WORKSHOP_STARTING_POSE))
        .andThen(new ExampleAuto(subDrivetrain));
  }

  public void resetToAutoPose() {
    Rotation2d desiredRotation = Rotation2d.kZero;

    try {
      desiredRotation = PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.getSelected().getName()).get(0)
          .getIdealStartingState().rotation();
      if (constField.isRedAlliance()) {
        desiredRotation = desiredRotation.plus(Rotation2d.k180deg);
      }
    } catch (Exception e) {
    }

    subDrivetrain.resetPoseToPose(new Pose2d(subDrivetrain.getPose().getTranslation(), desiredRotation));
  }

  private void configureAutoSelector() {
    autoChooser = AutoBuilder.buildAutoChooser("Four-Piece-Low");
    SmartDashboard.putData(autoChooser);
  }

  private void configureAutoBindings() {
    // i decorate my commands like a christmas tree
    // -- Named Commands --
    NamedCommands.registerCommand("PrepPlace",
        Commands.runOnce(() -> subStateMachine.tryState(RobotState.PREP_CORAL_L4))
            .until(() -> subStateMachine.getRobotState() == RobotState.PREP_CORAL_L4)
            .asProxy().withName("PrepPlace"));

    NamedCommands.registerCommand("PrepPlaceWithAlgae",
        Commands.runOnce(() -> subStateMachine.tryState(RobotState.PREP_CORAL_L4_WITH_ALGAE))
            .until(() -> subStateMachine.getRobotState() == RobotState.PREP_CORAL_L4_WITH_ALGAE)
            .asProxy());

    // -- Event Markers --
    EventTrigger prepL2 = new EventTrigger("PrepL2");
    prepL2
        .onTrue(new DeferredCommand(() -> subStateMachine.tryState(RobotState.PREP_CORAL_L1),
            Set.of(subStateMachine)).repeatedly()
            .until(() -> subStateMachine.getRobotState() == RobotState.PREP_CORAL_L1));

    EventTrigger prepPlace = new EventTrigger("PrepPlace");
    prepPlace
        .onTrue(new DeferredCommand(() -> subStateMachine.tryState(RobotState.PREP_CORAL_L4),
            Set.of(subStateMachine)).repeatedly()
            .until(() -> subStateMachine.getRobotState() == RobotState.PREP_CORAL_L4));
  }

  /**
   * Populates the selected AutoMap for your autonomous command.
   */
  private void selectAutoMap() {
    SELECTED_AUTO_PREP_MAP = configureAutoPrepMaps(autoChooser.getSelected().getName());
    SELECTED_AUTO_PREP_MAP_NAME = autoChooser.getSelected().getName();
  }

  /**
   * Configures the autonomous preparation maps based on the selected autonomous routine.
   * This method returns an array of pairs, where each pair consists of a {@link RobotState} 
   * and a {@link Pose2d} representing the robot's state and its target position on the field.
   * 
   * The method supports a variety of autonomous routines, each with its own predefined sequence 
   * of robot states and field positions. The positions are retrieved from the field configuration 
   * and are dynamically adjusted based on the alliance color (red or blue).
   * 
   * @param selectedAuto The name of the selected autonomous routine. This determines the sequence 
   *                     of robot states and positions to be used. Supported values include:
   *                     <ul>
   *                       <li>"Four_Piece_High" - A routine for scoring four pieces in high positions.</li>
   *                       <li>"Four_Piece_High_Double_Tickle" - A variation of the high scoring routine.</li>
   *                       <li>"Four_Piece_Low" - A routine for scoring four pieces in low positions.</li>
   *                       <li>"Four_Piece_High_Single_Tickle" - Another variation of the high scoring routine.</li>
   *                       <li>"Algae_Net" - A routine involving algae positions and net scoring.</li>
   *                       <li>"Ch_Algae_2.5_Net_Side" - A routine for handling algae and net scoring on one side.</li>
   *                       <li>"Ch_Algae_2.5_Net_To_KL" - A routine for handling algae and net scoring towards KL positions.</li>
   *                       <li>"Right_Algae_Net" - A routine for handling algae and net scoring on the right side.</li>
   *                       <li>"Ch_Algae_2.5_Processor_Side" - A routine for handling algae and processor scoring on one side.</li>
   *                       <li>"Ch_Algae_2.5_Processor_To_CD" - A routine for handling algae and processor scoring towards CD positions.</li>
   *                       <li>"Moo_High" - A high scoring routine with a specific sequence of positions.</li>
   *                       <li>"Moo_Low" - A low scoring routine with a specific sequence of positions.</li>
   *                     </ul>
   * 
   * @return An array of {@link Pair} objects, where each pair contains:
   *         <ul>
   *           <li>{@link RobotState} - The robot's state during the autonomous routine.</li>
   *           <li>{@link Pose2d} - The target position on the field for the robot.</li>
   *         </ul>
   *         The size of the array depends on the selected routine. If no valid routine is selected, 
   *         a default array with a single pair is returned, where the position is an empty {@link Pose2d}.
   * 
   * <p>Implementation Details:</p>
   * <ul>
   *   <li>The method uses predefined constants for robot states, such as {@code AUTO_PREP_CORAL_4}.</li>
   *   <li>Field positions and algae positions are dynamically retrieved based on the alliance color.</li>
   *   <li>Each case in the switch statement corresponds to a specific autonomous routine, defining 
   *       the sequence of states and positions.</li>
   *   <li>If the selected routine is not recognized, a default pair with an empty position is returned.</li>
   * </ul>
   * 
   * <p>Note:</p>
   * <ul>
   *   <li>Ensure that the field positions and algae positions are properly initialized before calling this method.</li>
   *   <li>Invalid indices in the field positions or algae positions lists may result in runtime exceptions.</li>
   * </ul>
   */
  private Pair<RobotState, Pose2d>[] configureAutoPrepMaps(String selectedAuto) {
    RobotState AUTO_PREP_CORAL_4 = RobotState.PREP_CORAL_L4;
    RobotState AUTO_PREP_CORAL_2 = RobotState.PREP_CORAL_L2;
    List<Pose2d> fieldPositions = constField.getReefPositions(constField.isRedAlliance()).get();
    List<Pose2d> algaePositions = constField.getAlgaePositionsClose(constField.isRedAlliance()).get();

    switch (selectedAuto) {
      case "Four_Piece_High":
        Pair<RobotState, Pose2d>[] fourPieceHigh = new Pair[4];
        fourPieceHigh[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(11)); // L
        fourPieceHigh[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(10)); // K
        fourPieceHigh[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(0)); // A
        fourPieceHigh[3] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(9)); // J
        return fourPieceHigh;
      case "Four_Piece_High_Double_Tickle":
        Pair<RobotState, Pose2d>[] fourPieceHighDoubleTickle = new Pair[4];
        fourPieceHighDoubleTickle[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(11)); // L
        fourPieceHighDoubleTickle[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(10)); // K
        fourPieceHighDoubleTickle[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(0)); // A
        fourPieceHighDoubleTickle[3] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(9)); // J
        return fourPieceHighDoubleTickle;
      case "Four_Piece_Low":
        Pair<RobotState, Pose2d>[] fourPieceLow = new Pair[4];
        fourPieceLow[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(2)); // C
        fourPieceLow[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(3)); // D
        fourPieceLow[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(1)); // B
        fourPieceLow[3] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(4)); // E
        return fourPieceLow;
      case "Four_Piece_High_Single_Tickle":
        Pair<RobotState, Pose2d>[] fourPieceHighSingleTickle = new Pair[4];
        fourPieceHighSingleTickle[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(11)); // L
        fourPieceHighSingleTickle[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(10)); // K
        fourPieceHighSingleTickle[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(0)); // A
        fourPieceHighSingleTickle[3] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(9)); // J
        return fourPieceHighSingleTickle;
      case "Algae_Net":
        Pair<RobotState, Pose2d>[] algaeNet = new Pair[3];
        algaeNet[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(7)); // H
        algaeNet[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(3)); // ALGAE GH
        algaeNet[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(4)); // ALGAE IJ
        return algaeNet;
      case "Ch_Algae_2.5_Net_Side":
        Pair<RobotState, Pose2d>[] algae3NetSide = new Pair[4];
        algae3NetSide[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(7)); // H
        algae3NetSide[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(3)); // ALGAE GH
        algae3NetSide[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(4)); // ALGAE IJ
        algae3NetSide[3] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(2)); // ALGAE EF
        return algae3NetSide;

      case "Ch_Algae_2.5_Net_To_KL":
        Pair<RobotState, Pose2d>[] algae3NetKL = new Pair[4];
        algae3NetKL[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(7)); // H
        algae3NetKL[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(3)); // ALGAE GH
        algae3NetKL[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(4)); // ALGAE IJ
        algae3NetKL[3] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(5)); // ALGAE KL
        return algae3NetKL;

      case "Right_Algae_Net":
        Pair<RobotState, Pose2d>[] rightAlgaeNet = new Pair[3];
        rightAlgaeNet[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(7)); // H
        rightAlgaeNet[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(3)); // ALGAE GH
        rightAlgaeNet[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(2)); // ALGAE EF
        return rightAlgaeNet;

      case "Ch_Algae_2.5_Processor_Side":
        Pair<RobotState, Pose2d>[] algae3ProcessorSide = new Pair[4];
        algae3ProcessorSide[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(7)); // H
        algae3ProcessorSide[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(3)); // ALGAE GH
        algae3ProcessorSide[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(2)); // ALGAE EF
        algae3ProcessorSide[3] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(4)); // ALGAE IJ
        return algae3ProcessorSide;

      case "Ch_Algae_2.5_Processor_To_CD":
        Pair<RobotState, Pose2d>[] algae3ProcessorCD = new Pair[4];
        algae3ProcessorCD[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(7)); // H
        algae3ProcessorCD[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(3)); // ALGAE GH
        algae3ProcessorCD[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(2)); // ALGAE EF
        algae3ProcessorCD[3] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, algaePositions.get(1)); // ALGAE CD
        return algae3ProcessorCD;

      case "Moo_High":
        Pair<RobotState, Pose2d>[] mooHigh = new Pair[4];
        mooHigh[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(9)); // j
        mooHigh[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(11)); // l
        mooHigh[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(10)); // k
        mooHigh[3] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(8)); // i
        return mooHigh;
      case "Moo_Low":
        Pair<RobotState, Pose2d>[] mooLow = new Pair[4];
        mooLow[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(4)); // E
        mooLow[1] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(2)); // C
        mooLow[2] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(3)); // D
        mooLow[3] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, fieldPositions.get(5)); // F
        return mooLow;
      default:
        Pair<RobotState, Pose2d>[] noAutoSelected = new Pair[1];
        noAutoSelected[0] = new Pair<RobotState, Pose2d>(AUTO_PREP_CORAL_4, new Pose2d());
        return noAutoSelected;
    }
  }

}
