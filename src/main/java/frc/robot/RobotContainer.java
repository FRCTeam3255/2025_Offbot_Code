// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;

import choreo.auto.AutoFactory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverState;
import frc.robot.Constants.constControllers;
import frc.robot.Constants.constField;
import frc.robot.Constants.constLED;
import frc.robot.Constants.constPoseDrive;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.States.None;
import frc.robot.commands.States.climbing.Climbing;
import frc.robot.commands.States.climbing.PrepClimb;
import frc.robot.commands.States.first_scoring_element.CleanHigh;
import frc.robot.commands.States.first_scoring_element.CleanLow;
import frc.robot.commands.States.first_scoring_element.IntakeAlgaeGround;
import frc.robot.commands.States.first_scoring_element.IntakeCoralGround;
import frc.robot.commands.States.first_scoring_element.IntakeCoralL1;
import frc.robot.commands.States.first_scoring_element.IntakeCoralStation;
import frc.robot.commands.States.hold_scoring_elements.HasAlgae;
import frc.robot.commands.States.hold_scoring_elements.HasCoral;
import frc.robot.commands.States.hold_scoring_elements.HasCoralAndAlgae;
import frc.robot.commands.States.prep_algae.PrepAlgaeZero;
import frc.robot.commands.States.prep_algae.PrepNet;
import frc.robot.commands.States.prep_algae.PrepNetWithCoral;
import frc.robot.commands.States.prep_algae.PrepProcessor;
import frc.robot.commands.States.prep_algae.PrepProcessorWithCoral;
import frc.robot.commands.States.prep_coral.PrepCoralL1;
import frc.robot.commands.States.prep_coral.PrepCoralL2;
import frc.robot.commands.States.prep_coral.PrepCoralL3;
import frc.robot.commands.States.prep_coral.PrepCoralL4;
import frc.robot.commands.States.prep_coral.PrepCoralWithAlgaeL2;
import frc.robot.commands.States.prep_coral.PrepCoralWithAlgaeL3;
import frc.robot.commands.States.prep_coral.PrepCoralWithAlgaeL4;
import frc.robot.commands.States.prep_coral.PrepCoralZero;
import frc.robot.commands.States.prep_coral.PrepCoralZeroWithAlgae;
import frc.robot.commands.States.scoring.ScoringAlgae;
import frc.robot.commands.States.scoring.ScoringAlgaeWithCoral;
import frc.robot.commands.States.scoring.ScoringCoral;
import frc.robot.commands.States.scoring.ScoringCoralWithAlgae;
import frc.robot.commands.States.scoring.ScoringL1Coral;
import frc.robot.commands.States.second_scoring_element.CleanHighWithCoral;
import frc.robot.commands.States.second_scoring_element.CleanLowWithCoral;
import frc.robot.commands.States.second_scoring_element.Ejecting;
import frc.robot.commands.States.second_scoring_element.IntakeAlgaeGroundWithCoral;
import frc.robot.commands.States.second_scoring_element.IntakeCoralGroundWithAlgae;
import frc.robot.commands.States.second_scoring_element.IntakeCoralStationWithAlgae;
import frc.robot.commands.Zeroing.ManualZeroLift;
import frc.robot.commands.Zeroing.ManualZeroPivot;
import frc.robot.commands.Zeroing.ManualZeroWrist;
import frc.robot.commands.Zeroing.StartingConfig;
import frc.robot.commands.driver_states.DriveManual;
import frc.robot.commands.driver_states.PoseDriving;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Motion;
import frc.robot.subsystems.RobotPoses;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.Rotors;
import frc.robot.subsystems.Vision;

@Logged
public class RobotContainer {

  @NotLogged
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  private AutoFactory autoFactory;

  public static final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  public static final SN_XboxController conOperator = new SN_XboxController(mapControllers.OPERATOR_USB);

  public static final Drivetrain subDrivetrain = new Drivetrain();
  public static final Rotors subRotors = new Rotors();
  public static final Motion subMotion = new Motion();
  public static final LED subLED = new LED();
  public static final RobotPoses robotPoseInstance = new RobotPoses(subDrivetrain, subMotion, subRotors);
  public static final Vision subVision = new Vision();
  public final RobotPoses robotPoses = robotPoseInstance;

  // Robot state tracking
  public static RobotState currentRobotState = RobotState.NONE;

  // Driver state tracking
  public static DriverState currentDriverState = DriverState.MANUAL;

  public static void setDriverState(DriverState state) {
    currentDriverState = state;
  }

  public static DriverState getDriverState() {
    return currentDriverState;
  }

  public RobotState getCurrentRobotState() {
    return currentRobotState;
  }

  public DriverState getCurrentDriverState() {
    return currentDriverState;
  }

  public static void setRobotState(RobotState state) {
    currentRobotState = state;
  }

  public static RobotState getRobotState() {
    return currentRobotState;
  }

  public static boolean inCleaningState() {
    return currentRobotState == RobotState.CLEAN_HIGH
        || currentRobotState == RobotState.CLEAN_LOW
        || currentRobotState == RobotState.CLEAN_HIGH_WITH_CORAL
        || currentRobotState == RobotState.CLEAN_LOW_WITH_CORAL;
  }

  public Command manualZeroLift = new ManualZeroLift(subMotion, subLED).ignoringDisable(true);
  public Command manualZeroPivot = new ManualZeroPivot(subMotion, subLED).ignoringDisable(true);
  public Command manualZeroWrist = new ManualZeroWrist(subMotion, subLED).ignoringDisable(true);
  public Command startingCofig = new StartingConfig(subMotion, subLED).ignoringDisable(true);

  // private final BooleanSupplier isReadyToScoreReef = ;
  // private final BooleanSupplier isReadyToScoreNet = ;
  private final Trigger isReadyToScoreReefFeedback = new Trigger(() -> (subDrivetrain.atLastDesiredFieldPosition()
      && subMotion.atLastDesiredMechPosition()));
  private final Trigger isReadyToScoreNetFeedback = new Trigger(() -> (subDrivetrain.atLastDesiredFieldPosition()));
  private final Trigger hasCoralTrigger = new Trigger(() -> subRotors.hasCoral() && !subRotors.hasAlgae());
  private final Trigger hasAlgaeTrigger = new Trigger(() -> !subRotors.hasCoral() && subRotors.hasAlgae());
  private final Trigger hasBothTrigger = new Trigger(() -> subRotors.hasCoral() && subRotors.hasAlgae());
  private final Trigger isInCleaningStates = new Trigger(() -> inCleaningState());
  private final Trigger hasCoralL1Trigger = new Trigger(() -> subRotors.hasL1Coral());
  private final Trigger isCageLatchedTrigger = new Trigger(() -> subRotors.isCageLatched());
  private final Trigger isInCSAutoDriveState = new Trigger(
      () -> currentDriverState == DriverState.CORAL_STATION_AUTO_DRIVING_FAR
          || currentDriverState == DriverState.CORAL_STATION_AUTO_DRIVING_CLOSE);
  private final Trigger isInProcessorAutoDriveState = new Trigger(
      () -> currentDriverState == DriverState.PROCESSOR_AUTO_DRIVING);
  private final Trigger isInPrepL2States = new Trigger(
      () -> getRobotState() == RobotState.PREP_CORAL_L2
          || getRobotState() == RobotState.PREP_CORAL_L2_WITH_ALGAE);
  private final Trigger isInClimbState = new Trigger(
      () -> getRobotState() == RobotState.CLIMBING
          || getRobotState() == RobotState.PREP_CLIMB);

  private Command nonProcSide4Coral;
  private Command procSide4Coral;
  private Command mid1Coral;
  private Command midAlgae;

  public RobotContainer() {
    RobotController.setBrownoutVoltage(5.5);
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    subDrivetrain.setDefaultCommand(new DriveManual());

    configDriverBindings();
    configOperatorBindings();
    configAutos();
    configFeedback();

    autoChooser.onChange(selectedAuto -> {
      String pose = "default_pose"; // Initialize with a default value
      if (selectedAuto == nonProcSide4Coral) {
        pose = "top_ji";
      } else if (selectedAuto == procSide4Coral) {
        pose = "proc_ef";
      } else if (selectedAuto == mid1Coral || selectedAuto == midAlgae) {
        pose = "mid_gh";
      }
      autoFactory.resetOdometry(pose).ignoringDisable(true).schedule();
    });

    subDrivetrain.resetModulesToAbsolute();
  }

  public void configAutos() {
    autoFactory = new AutoFactory(
        subDrivetrain::getPose, // A function that returns the current robot pose
        subDrivetrain::resetPoseToPose, // A function that resets the current robot pose to the provided Pose2d
        subDrivetrain::followTrajectory, // The drive subsystem trajectory follower
        true, // If alliance flipping should be enabled
        subDrivetrain // The drive subsystem
    );

    nonProcSide4Coral = Commands.sequence(
        ScoreAndCollect("top_ji", "ji_cs", new PoseDriving(constPoseDrive.CORAL_REEF_RIGHT),
            new PrepCoralL4()),
        ScoreAndCollect("cs_lk", "lk_cs", new PoseDriving(constPoseDrive.CORAL_REEF_RIGHT),
            new PrepCoralL4()),
        ScoreAndCollect("cs_lk", "lk_cs", new PoseDriving(constPoseDrive.CORAL_REEF_LEFT), new PrepCoralL4()),
        ScoreAndCollect("cs_ab", "ab_cs", new PoseDriving(constPoseDrive.CORAL_REEF_LEFT),
            new PrepCoralL4()));

    procSide4Coral = Commands.sequence(
        ScoreAndCollect("proc_ef", "ef_cs", new PoseDriving(constPoseDrive.CORAL_REEF_RIGHT),
            new PrepCoralL4()),
        ScoreAndCollect("cs_cd", "cd_cs", new PoseDriving(constPoseDrive.CORAL_REEF_RIGHT),
            new PrepCoralL4()),
        ScoreAndCollect("cs_cd", "cd_cs", new PoseDriving(constPoseDrive.CORAL_REEF_LEFT), new PrepCoralL4()),
        ScoreAndCollect("proc_cs_ab", "ab_proc_cs", new PoseDriving(constPoseDrive.CORAL_REEF_LEFT),
            new PrepCoralL4()));

    mid1Coral = Commands.sequence(
        Score("mid_gh", new PoseDriving(constPoseDrive.CORAL_REEF_LEFT), new PrepCoralL4()));

    midAlgae = Commands.sequence(
        Score("mid_gh", new PoseDriving(constPoseDrive.CORAL_REEF_LEFT), new PrepCoralL4()),
        FirstCleanAndScore("gh_net", new CleanLow()),
        CleanAndScore("net_ji", "ji_net", new CleanHigh()),
        CleanAndScore("net_ef", "ef_net", new CleanHigh()),
        runPath("net_off_startingline")); // FORGOT TO DO AS PROXY ON RUNPATH

    autoChooser.addOption("4 Coral - Non-Processor Side", nonProcSide4Coral);
    autoChooser.addOption("4 Coral - Processor Side", procSide4Coral);
    autoChooser.addOption("1 Coral - Mid", mid1Coral);
    autoChooser.addOption("3 Algae - Mid", midAlgae);

    SmartDashboard.putData("AutoChooser", autoChooser);
  }

  Command ScoreAndCollect(String startPath, String endPath, Command reef_auto_drive_branch, Command try_prep_coral_l) {
    return Commands.sequence(
        Commands.runOnce(() -> RobotContainer.setRobotState(RobotState.HAS_CORAL)),
        runPath(startPath),
        reef_auto_drive_branch.alongWith(
            Commands.waitSeconds(0.3).andThen(
                try_prep_coral_l))
            .withTimeout(2),
        new ScoringCoral().withTimeout(0.6),
        new None().withTimeout(0.05),
        runPath(endPath),
        new PoseDriving(constPoseDrive.CORAL_STATION_FAR).withDeadline(new IntakeCoralStation()).withTimeout(10));
  }

  Command Score(String startPath, Command reef_auto_drive_branch, Command try_prep_coral_l) {
    return Commands.sequence(
        Commands.runOnce(() -> RobotContainer.setRobotState(RobotState.HAS_CORAL)),
        runPath(startPath),
        reef_auto_drive_branch.alongWith(
            Commands.waitSeconds(0.3).andThen(
                try_prep_coral_l))
            .withTimeout(2),
        new ScoringCoral().withTimeout(0.5),
        new None().withTimeout(0.1));
  }

  Command CleanAndScore(String startPath, String endPath, Command try_clean_lv) {
    return Commands.sequence(
        runPath(startPath),
        new PoseDriving(constPoseDrive.ALGAE_REEF).withDeadline(
            try_clean_lv).withTimeout(4),
        Commands.runOnce(() -> RobotContainer.setRobotState(RobotState.HAS_ALGAE)),
        runPath(endPath),
        new PoseDriving(constPoseDrive.NET).alongWith(
            Commands.waitSeconds(0.3).andThen(
                new PrepNet()))
            .withTimeout(1.5),
        new ScoringAlgae().withTimeout(0.5),
        new None().withTimeout(0.05));
  }

  Command FirstCleanAndScore(String endPath, Command try_clean_lv) {
    return Commands.sequence(
        new PoseDriving(constPoseDrive.ALGAE_REEF).withTimeout(0.7).andThen(
            new PoseDriving(constPoseDrive.ALGAE_REEF).withDeadline(
                try_clean_lv.withTimeout(4))),
        Commands.runOnce(() -> RobotContainer.setRobotState(RobotState.HAS_ALGAE)),
        runPath(endPath),
        new PoseDriving(constPoseDrive.NET).alongWith(
            Commands.waitSeconds(0.3).andThen(
                new PrepNet()))
            .withTimeout(1.5),
        new ScoringAlgae().withTimeout(0.5),
        new None().withTimeout(0.05));
  }

  Command runPath(String pathName) {
    return autoFactory.trajectoryCmd(pathName)
        .alongWith(Commands.runOnce(() -> RobotContainer.setDriverState(DriverState.CHOREO)));
  }

  private void configDriverBindings() {
    conDriver.btn_B.onTrue(Commands.runOnce(() -> subDrivetrain.resetModulesToAbsolute()));
    conDriver.btn_North
        .onTrue(Commands
            .runOnce(() -> subDrivetrain.resetPoseToPose(constField.RESET_POS)));

    conDriver.btn_LeftTrigger
        .whileTrue(new PoseDriving(constPoseDrive.CORAL_REEF_LEFT)).and(isInCleaningStates.negate())
        .onFalse(new DriveManual());

    conDriver.btn_RightTrigger.and(isInCleaningStates.negate())
        .whileTrue(new PoseDriving(constPoseDrive.CORAL_REEF_RIGHT))
        .onFalse(new DriveManual());

    conDriver.btn_LeftTrigger.and(isInCleaningStates)
        .whileTrue(new PoseDriving(constPoseDrive.ALGAE_REEF))
        .onFalse(new DriveManual());

    conDriver.btn_LeftTrigger.and(isInCleaningStates)
        .whileTrue(new PoseDriving(constPoseDrive.ALGAE_REEF))
        .onFalse(new DriveManual());

    conDriver.btn_RightTrigger.and(isInCleaningStates)
        .whileTrue(new PoseDriving(constPoseDrive.ALGAE_REEF))
        .onFalse(new DriveManual());

    conDriver.btn_RightTrigger.and(isInCleaningStates)
        .whileTrue(new PoseDriving(constPoseDrive.ALGAE_REEF))
        .onFalse(new DriveManual());

    conDriver.btn_X
        .whileTrue(new PoseDriving(constPoseDrive.CORAL_STATION_FAR))
        .onFalse(new DriveManual());

    conDriver.btn_B
        .whileTrue(new PoseDriving(constPoseDrive.CORAL_STATION_CLOSE))
        .onFalse(new DriveManual());

    conDriver.btn_East
        .whileTrue(new PoseDriving(constPoseDrive.PROCESSOR))
        .onFalse(new DriveManual());

    conDriver.btn_East
        .whileTrue(new PoseDriving(constPoseDrive.PROCESSOR))
        .onFalse(new DriveManual());

    conDriver.btn_South
        .whileTrue(new PoseDriving(constPoseDrive.CAGE))
        .onFalse(new DriveManual());

    conDriver.btn_LeftBumper
        .whileTrue(new PoseDriving(constPoseDrive.NET))
        .onFalse(new DriveManual());

    conDriver.btn_LeftBumper
        .whileTrue(new PoseDriving(constPoseDrive.NET))
        .onFalse(new DriveManual());

    conDriver.btn_Start
        .onTrue(new PrepClimb());

    conDriver.btn_Y
        .whileTrue(new Climbing());

    isInCSAutoDriveState
        .whileTrue(new IntakeCoralStation())
        .onFalse(new None());

    isInProcessorAutoDriveState
        .whileTrue(new PrepProcessor())
        .whileTrue(new PrepProcessorWithCoral());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();

  }

  private void configOperatorBindings() {
    // Add operator bindings here if needed
    conOperator.btn_LeftTrigger
        .whileTrue(new IntakeCoralGround())
        .whileTrue(new IntakeCoralGroundWithAlgae())
        .onFalse(new None())
        .onFalse(new HasAlgae());

    conOperator.btn_LeftBumper
        .whileTrue(new IntakeAlgaeGround())
        .whileTrue(new IntakeAlgaeGroundWithCoral())
        .onFalse(new None())
        .onFalse(new HasCoral());

    conOperator.btn_RightTrigger
        .whileTrue(new ScoringCoral())
        .whileTrue(new ScoringAlgae())
        .whileTrue(new ScoringAlgaeWithCoral())
        .whileTrue(new ScoringCoralWithAlgae())
        .whileTrue(new ScoringL1Coral())
        .onFalse(new None())
        .onFalse(new HasCoral())
        .onFalse(new HasAlgae());

    conOperator.btn_RightBumper
        .whileTrue(new IntakeCoralStation())
        .whileTrue(new IntakeCoralStationWithAlgae())
        .onFalse(new None())
        .onFalse(new HasAlgae());

    conOperator.btn_A
        .whileTrue(new IntakeCoralL1())
        .onFalse(new None());

    conOperator.btn_B
        .onTrue(new PrepCoralL3())
        .onTrue(new PrepCoralWithAlgaeL3());

    conOperator.btn_X
        .onTrue(new PrepCoralL2())
        .onTrue(new PrepCoralWithAlgaeL2());

    conOperator.btn_Y
        .onTrue(new PrepCoralL4())
        .onTrue(new PrepCoralWithAlgaeL4());

    conOperator.btn_LeftStick
        .whileTrue(new Ejecting())
        .onFalse(new None());

    conOperator.btn_RightStick
        .onTrue(new PrepCoralZero())
        .onTrue(new PrepCoralZeroWithAlgae())
        .onTrue(new PrepAlgaeZero());
    conOperator.btn_RightStick.and(isInClimbState).onTrue(new None());

    conOperator.btn_North
        .onTrue(new PrepNet())
        .onTrue(new PrepNetWithCoral());

    conOperator.btn_South
        .onTrue(new PrepProcessor())
        .onTrue(new PrepProcessorWithCoral());

    conOperator.btn_East
        .whileTrue(new CleanHigh())
        .whileTrue(new CleanHighWithCoral())
        .onFalse(new None())
        .onFalse(new HasCoral());

    conOperator.btn_West
        .whileTrue(new CleanLow())
        .whileTrue(new CleanLowWithCoral())
        .onFalse(new None())
        .onFalse(new HasCoral());

    conOperator.btn_Start

        .onTrue(new HasCoral())
        .onTrue(new PrepCoralL1());

    conOperator.btn_Back
        .onTrue(new HasAlgae());

    hasCoralTrigger.debounce(0.1)
        .whileTrue(new HasCoral());

    hasAlgaeTrigger// debounce(0.2).and(conOperator.btn_West.negate()).and(conOperator.btn_East.negate())
        .whileTrue(new HasAlgae());

    hasBothTrigger
        .whileTrue(new HasCoralAndAlgae());

    hasCoralL1Trigger.debounce(0.1)
        .whileTrue(new PrepCoralL1());

    isCageLatchedTrigger.debounce(0.4)
        .onTrue(new Climbing());
  }

  public void configFeedback() {
    isReadyToScoreReefFeedback
        .onTrue(Commands.runOnce(() -> subLED.setLED(constLED.READY_TO_SHOOT_ANIMATION, 0)))
        .whileTrue(
            Commands.runOnce(() -> conOperator.setRumble(RumbleType.kBothRumble, constControllers.OPERATOR_RUMBLE)))
        .onFalse(Commands.runOnce(() -> conOperator.setRumble(RumbleType.kBothRumble, 0)))
        .onFalse(Commands.runOnce(() -> subLED.clearAnimation()));
    isReadyToScoreNetFeedback
        .onTrue(Commands.runOnce(() -> subLED.setLED(constLED.READY_TO_SHOOT_ANIMATION, 0)))
        .whileTrue(
            Commands.runOnce(() -> conOperator.setRumble(RumbleType.kBothRumble, constControllers.OPERATOR_RUMBLE)))
        .onFalse(Commands.runOnce(() -> conOperator.setRumble(RumbleType.kBothRumble, 0)))
        .onFalse(Commands.runOnce(() -> subLED.clearAnimation()));
  }

  public boolean allZeroed() {
    return subMotion.hasLiftZeroed && subMotion.hasPivotZeroed && subMotion.hasWristZeroed;
  }

  public Command AddVisionMeasurement() {
    return new AddVisionMeasurement(subDrivetrain, subVision)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).ignoringDisable(true);
  }
}
