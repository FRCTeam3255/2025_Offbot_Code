// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;
import com.pathplanner.lib.events.EventTrigger;

import choreo.auto.AutoFactory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.constControllers;
import frc.robot.Constants.constField;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.Zeroing.ManualZeroLift;
import frc.robot.commands.Zeroing.ManualZeroPivot;
import frc.robot.commands.Zeroing.ManualZeroWrist;
import frc.robot.commands.Zeroing.StartingConfig;
import frc.robot.commands.driver_states.DriveManual;
import frc.robot.subsystems.DriverStateMachine;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Motion;
import frc.robot.subsystems.RobotPoses;
import frc.robot.subsystems.Rotors;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.Vision;

@Logged
public class RobotContainer {
  @NotLogged
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final AutoFactory autoFactory;

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

  public RobotContainer() {
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    subDrivetrain
        .setDefaultCommand(new DriveManual(
            subDrivetrain, subDriverStateMachine, conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX));

    configDriverBindings();
    configOperatorBindings();

    subDrivetrain.resetModulesToAbsolute();

    autoFactory = new AutoFactory(
        subDrivetrain::getPose, // A function that returns the current robot pose
        subDrivetrain::resetPoseToPose, // A function that resets the current robot pose to the provided Pose2d
        subDrivetrain::followTrajectory, // The drive subsystem trajectory follower
        true, // If alliance flipping should be enabled
        subDrivetrain // The drive subsystem
    );
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

  public Command getAutonomousCommand() {
    return Commands.sequence(
        autoFactory.resetOdometry("top_ji"),
        Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.HAS_CORAL)),
        autoFactory.trajectoryCmd("top_ji"),
        Commands.waitSeconds(0.5),
        TRY_PREP_CORAL_L4,
        Commands.waitSeconds(0.5),
        REEF_AUTO_DRIVING_RIGHT,
        Commands.waitSeconds(0.5),
        TRY_SCORING_CORAL,
        Commands.waitSeconds(0.5),
        autoFactory.trajectoryCmd("ji_cs"),
        Commands.waitSeconds(0.5),
        CORAL_STATION_AUTO_DRIVING_CLOSE,
        Commands.waitSeconds(0.5),
        TRY_INTAKE_CORAL_STATION

    );

  }

  private void configOperatorBindings() {
    // Add operator bindings here if needed
    conOperator.btn_LeftTrigger.or(new EventTrigger("INTAKE_CORAL_GROUND_TRIGGER"))
        .whileTrue(TRY_INTAKE_CORAL_GROUND)
        .whileTrue(TRY_INTAKE_CORAL_GROUND_WITH_ALGAE)
        .onFalse(TRY_NONE)
        .onFalse(TRY_HAS_ALGAE);

    conOperator.btn_LeftBumper.or(new EventTrigger("INTAKE_ALGAE_GROUND_TRIGGER"))
        .whileTrue(TRY_INTAKE_ALGAE_GROUND)
        .whileTrue(TRY_INTAKE_ALGAE_GROUND_WITH_CORAL)
        .onFalse(TRY_NONE)
        .onFalse(TRY_HAS_CORAL);

    conOperator.btn_RightTrigger.or(new EventTrigger("SCORING_TRIGGER"))
        .whileTrue(TRY_SCORING_CORAL)
        .whileTrue(TRY_SCORING_ALGAE)
        .whileTrue(TRY_SCORING_ALGAE_WITH_CORAL)
        .whileTrue(TRY_SCORING_CORAL_WITH_ALGAE)
        .whileTrue(TRY_SCORING_CORAL_L1)
        .onFalse(TRY_NONE)
        .onFalse(TRY_HAS_CORAL)
        .onFalse(TRY_HAS_ALGAE);

    conOperator.btn_RightBumper.or(new EventTrigger("INTAKE_CORAL_STATION_TRIGGER"))
        .whileTrue(TRY_INTAKE_CORAL_STATION)
        .whileTrue(TRY_INTAKE_CORAL_STATION_WITH_ALGAE)
        .onFalse(TRY_NONE)
        .onFalse(TRY_HAS_ALGAE);

    conOperator.btn_A.or(new EventTrigger("PREP_CORAL_L1_TRIGGER"))
        .whileTrue(TRY_INTAKE_CORAL_L1)
        .onFalse(TRY_NONE);

    conOperator.btn_B.or(new EventTrigger("PREP_CORAL_L3_TRIGGER"))
        .onTrue(TRY_PREP_CORAL_L3)
        .onTrue(TRY_PREP_CORAL_L3_WITH_ALGAE);

    conOperator.btn_X.or(new EventTrigger("PREP_CORAL_L2_TRIGGER"))
        .onTrue(TRY_PREP_CORAL_L2)
        .onTrue(TRY_PREP_CORAL_L2_WITH_ALGAE);

    conOperator.btn_Y.or(new EventTrigger("PREP_CORAL_L4_TRIGGER"))
        .onTrue(TRY_PREP_CORAL_L4)
        .onTrue(TRY_PREP_CORAL_L4_WITH_ALGAE);

    conOperator.btn_LeftStick.or(new EventTrigger("EJECTING_TRIGGER"))
        .whileTrue(TRY_EJECTING)
        .onFalse(TRY_NONE);

    conOperator.btn_RightStick.or(new EventTrigger("PREP_CORAL_ZERO_TRIGGER"))
        .onTrue(TRY_PREP_CORAL_ZERO)
        .onTrue(TRY_PREP_CORAL_ZERO_WITH_ALGAE)
        .onTrue(TRY_PREP_ALGAE_ZERO);

    conOperator.btn_North.or(new EventTrigger("PREP_ALGAE_NET_TRIGGER"))
        .onTrue(TRY_PREP_ALGAE_NET)
        .onTrue(TRY_PREP_ALGAE_NET_WITH_CORAL);

    conOperator.btn_South.or(new EventTrigger("PREP_ALGAE_PROCESSOR_TRIGGER"))
        .onTrue(TRY_PREP_ALGAE_PROCESSOR)
        .onTrue(TRY_PREP_ALGAE_PROCESSOR_WITH_CORAL);

    conOperator.btn_East.or(new EventTrigger("CLEAN_HIGH_TRIGGER"))
        .whileTrue(TRY_CLEAN_HIGH)
        .whileTrue(TRY_CLEAN_HIGH_WITH_CORAL)
        .onFalse(TRY_NONE)
        .onFalse(TRY_HAS_CORAL);

    conOperator.btn_West.or(new EventTrigger("CLEAN_LOW_TRIGGER"))
        .whileTrue(TRY_CLEAN_LOW)
        .whileTrue(TRY_CLEAN_LOW_WITH_CORAL)
        .onFalse(TRY_NONE)
        .onFalse(TRY_HAS_CORAL);

    conOperator.btn_Start.or(new EventTrigger("HAS_CORAL_OVERRIDE_TRIGGER"))

        .onTrue(HAS_CORAL_OVERRIDE)
        .onTrue(HAS_CORAL_L1_OVERRIDE);

    conOperator.btn_Back.or(new EventTrigger("HAS_ALGAE_OVERRIDE_TRIGGER"))
        .onTrue(HAS_ALGAE_OVERRIDE);

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
}
