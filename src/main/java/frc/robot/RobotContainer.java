// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.constControllers;
import frc.robot.Constants.constField;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.*;
import frc.robot.commands.States.*;
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
import frc.robot.commands.States.prep_coral.PrepCoralLv;
import frc.robot.commands.States.prep_coral.PrepCoralWithAlgae;
import frc.robot.commands.States.scoring.ScoringAlgae;
import frc.robot.commands.States.scoring.ScoringAlgaeWithCoral;
import frc.robot.commands.States.scoring.ScoringCoral;
import frc.robot.commands.States.scoring.ScoringCoralWithAlgae;
import frc.robot.commands.States.scoring.ScoringL1Coral;
import frc.robot.commands.States.second_scoring_element.CleanHighWithCoral;
import frc.robot.commands.States.second_scoring_element.CleanLowWithCoral;
import frc.robot.commands.States.second_scoring_element.Ejecting;
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
  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  private final SN_XboxController conOperator = new SN_XboxController(mapControllers.OPERATOR_USB);

  private final Drivetrain subDrivetrain = Drivetrain.getInstance();
  private final Rotors subRotors = Rotors.getInstance();
  private final Motion subMotion = Motion.getInstance();
  private final LED subLED = LED.getInstance();
  private final StateMachine subStateMachine = StateMachine.getInstance();
  private final DriverStateMachine subDriverStateMachine = DriverStateMachine.getInstance();
  private final RobotPoses robotPose = RobotPoses.getInstance();
  private final Vision subVision = Vision.getInstance();

  public Command manualZeroLift = new ManualZeroLift(subMotion, subLED).ignoringDisable(true);
  public Command manualZeroPivot = new ManualZeroPivot(subMotion, subLED).ignoringDisable(true);
  public Command manualZeroWrist = new ManualZeroWrist(subMotion, subLED).ignoringDisable(true);
  public Command startingCofig = new StartingConfig().ignoringDisable(true);

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


  // STATES
  public final Trigger noneTrigger = new Trigger(() -> getRobotState() == RobotState.NONE).whileTrue(new None());
  public final Trigger prepClimbTrigger = new Trigger(() -> getRobotState() == RobotState.PREP_CLIMB).whileTrue(new PrepClimb());
  public final Trigger climbingTrigger = new Trigger(() -> getRobotState() == RobotState.CLIMBING).whileTrue(new Climbing());
  public final Trigger prepCoralZeroTrigger = new Trigger(() -> getRobotState() == RobotState.PREP_CORAL_ZERO).whileTrue(new PrepCoralLv(0));
  public final Trigger prepCoralL1Trigger = new Trigger(() -> getRobotState() == RobotState.PREP_CORAL_L1).whileTrue(new PrepCoralLv(1));
  public final Trigger prepCoralL2Trigger = new Trigger(() -> getRobotState() == RobotState.PREP_CORAL_L2).whileTrue(new PrepCoralLv(2));
  public final Trigger prepCoralL3Trigger = new Trigger(() -> getRobotState() == RobotState.PREP_CORAL_L3).whileTrue(new PrepCoralLv(3));
  public final Trigger prepCoralL4Trigger = new Trigger(() -> getRobotState() == RobotState.PREP_CORAL_L4).whileTrue(new PrepCoralLv(4));
  public final Trigger prepCoralL2WithAlgaeTrigger = new Trigger(() -> getRobotState() == RobotState.PREP_CORAL_L2_WITH_ALGAE).whileTrue(new PrepCoralWithAlgae(2));
  public final Trigger prepCoralL3WithAlgaeTrigger = new Trigger(() -> getRobotState() == RobotState.PREP_CORAL_L3_WITH_ALGAE).whileTrue(new PrepCoralWithAlgae(3));
  public final Trigger prepCoralL4WithAlgaeTrigger = new Trigger(() -> getRobotState() == RobotState.PREP_CORAL_L4_WITH_ALGAE).whileTrue(new PrepCoralWithAlgae(4));
  public final Trigger prepCoralZeroWithAlgaeTrigger = new Trigger(() -> getRobotState() == RobotState.PREP_CORAL_ZERO_WITH_ALGAE).whileTrue(new PrepCoralWithAlgae(0));
  public final Trigger prepAlgaeNetTrigger = new Trigger(() -> getRobotState() == RobotState.PREP_ALGAE_NET).whileTrue(new PrepNet());
  public final Trigger prepAlgaeProcessorTrigger = new Trigger(() -> getRobotState() == RobotState.PREP_ALGAE_PROCESSOR).whileTrue(new PrepProcessor());
  public final Trigger prepAlgaeZeroTrigger = new Trigger(() -> getRobotState() == RobotState.PREP_ALGAE_ZERO).whileTrue(new PrepAlgaeZero());
  public final Trigger prepAlgaeNetWithCoralTrigger = new Trigger(() -> getRobotState() == RobotState.PREP_ALGAE_NET_WITH_CORAL).whileTrue(new PrepNetWithCoral());
  public final Trigger prepAlgaeProcessorWithCoralTrigger = new Trigger(() -> getRobotState() == RobotState.PREP_ALGAE_PROCESSOR_WITH_CORAL).whileTrue(new PrepProcessorWithCoral());
  // public final Trigger hasCoralTrigger = new Trigger(() -> getRobotState() == RobotState.HAS_CORAL).whileTrue(new HasCoral());
  // public final Trigger hasAlgaeTrigger = new Trigger(() -> getRobotState() == RobotState.HAS_ALGAE).whileTrue(new HasAlgae());
  public final Trigger hasCoralAndAlgaeTrigger = new Trigger(() -> getRobotState() == RobotState.HAS_CORAL_AND_ALGAE).whileTrue(new HasCoralAndAlgae());
  public final Trigger scoringCoralTrigger = new Trigger(() -> getRobotState() == RobotState.SCORING_CORAL).whileTrue(new ScoringCoral());
  public final Trigger scoringAlgaeTrigger = new Trigger(() -> getRobotState() == RobotState.SCORING_ALGAE).whileTrue(new ScoringAlgae());
  public final Trigger scoringCoralL1Trigger = new Trigger(() -> getRobotState() == RobotState.SCORING_CORAL_L1).whileTrue(new ScoringL1Coral());
  public final Trigger cleanHighTrigger = new Trigger(() -> getRobotState() == RobotState.CLEAN_HIGH).whileTrue(new CleanHigh());
  public final Trigger cleanLowTrigger = new Trigger(() -> getRobotState() == RobotState.CLEAN_LOW).whileTrue(new CleanLow());
  public final Trigger intakeCoralStationTrigger = new Trigger(() -> getRobotState() == RobotState.INTAKE_CORAL_STATION).whileTrue(new IntakeCoralStation());
  public final Trigger intakeAlgaeGroundTrigger = new Trigger(() -> getRobotState() == RobotState.INTAKE_ALGAE_GROUND).whileTrue(new IntakeAlgaeGround());
  public final Trigger intakeCoralL1Trigger = new Trigger(() -> getRobotState() == RobotState.INTAKE_CORAL_L1).whileTrue(new IntakeCoralL1());
  public final Trigger ejectingTrigger = new Trigger(() -> getRobotState() == RobotState.EJECTING).whileTrue(new Ejecting());
  public final Trigger scoringAlgaeWithCoralTrigger = new Trigger(() -> getRobotState() == RobotState.SCORING_ALGAE_WITH_CORAL).whileTrue(new ScoringAlgaeWithCoral());
  public final Trigger scoringCoralWithAlgaeTrigger = new Trigger(() -> getRobotState() == RobotState.SCORING_CORAL_WITH_ALGAE).whileTrue(new ScoringCoralWithAlgae());
  public final Trigger cleanHighWithCoralTrigger = new Trigger(() -> getRobotState() == RobotState.CLEAN_HIGH_WITH_CORAL).whileTrue(new CleanHighWithCoral());
  public final Trigger cleanLowWithCoralTrigger = new Trigger(() -> getRobotState() == RobotState.CLEAN_LOW_WITH_CORAL).whileTrue(new CleanLowWithCoral());
  public final Trigger intakeCoralGroundTrigger = new Trigger(() -> getRobotState() == RobotState.INTAKE_CORAL_GROUND).whileTrue(new IntakeCoralGround());



  public RobotContainer() {
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    subDrivetrain
        .setDefaultCommand(new DriveManual(
            conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX));
 
    configDriverBindings();
    configOperatorBindings();

    subDrivetrain.resetModulesToAbsolute();
  }

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
        .onTrue(new TRY_STATE(RobotState.PREP_CLIMB));

    conDriver.btn_Y
        .whileTrue(new TRY_STATE(RobotState.CLIMBING));

    isInCSAutoDriveState
        .whileTrue(new TRY_STATE(RobotState.INTAKE_CORAL_STATION))
        .onFalse(new TRY_STATE(RobotState.NONE));

    isInProcessorAutoDriveState
        .whileTrue(new TRY_STATE(RobotState.PREP_ALGAE_PROCESSOR))
        .whileTrue(new TRY_STATE(RobotState.PREP_ALGAE_PROCESSOR_WITH_CORAL));
  }

  public Command getAutonomousCommand() {
    return Commands.runOnce(() -> subDrivetrain.resetPoseToPose(Constants.constField.WORKSHOP_STARTING_POSE))
        .andThen(new ExampleAuto(subDrivetrain));
  }

  private void configOperatorBindings() {
    conOperator.btn_LeftTrigger
        .whileTrue(new TRY_STATE(RobotState.INTAKE_CORAL_GROUND))
        .whileTrue(new TRY_STATE(RobotState.INTAKE_CORAL_GROUND_WITH_ALGAE))
        .onFalse(new TRY_STATE(RobotState.NONE))
        .onFalse(new TRY_STATE(RobotState.HAS_ALGAE));

    conOperator.btn_LeftBumper
        .whileTrue(new TRY_STATE(RobotState.INTAKE_ALGAE_GROUND))
        .whileTrue(new TRY_STATE(RobotState.INTAKE_ALGAE_GROUND_WITH_CORAL))
        .onFalse(new TRY_STATE(RobotState.NONE))
        .onFalse(new TRY_STATE(RobotState.HAS_CORAL));

    conOperator.btn_RightTrigger
        .whileTrue(new TRY_STATE(RobotState.SCORING_CORAL))
        .whileTrue(new TRY_STATE(RobotState.SCORING_ALGAE))
        .whileTrue(new TRY_STATE(RobotState.SCORING_ALGAE_WITH_CORAL))
        .whileTrue(new TRY_STATE(RobotState.SCORING_CORAL_WITH_ALGAE))
        .whileTrue(new TRY_STATE(RobotState.SCORING_CORAL_L1))
        .onFalse(new TRY_STATE(RobotState.NONE))
        .onFalse(new TRY_STATE(RobotState.HAS_CORAL))
        .onFalse(new TRY_STATE(RobotState.HAS_ALGAE));

    conOperator.btn_RightBumper
        .whileTrue(new TRY_STATE(RobotState.INTAKE_CORAL_STATION))
        .whileTrue(new TRY_STATE(RobotState.INTAKE_CORAL_STATION_WITH_ALGAE))
        .onFalse(new TRY_STATE(RobotState.NONE))
        .onFalse(new TRY_STATE(RobotState.HAS_ALGAE));

    conOperator.btn_A
        .whileTrue(new TRY_STATE(RobotState.INTAKE_CORAL_L1))
        .onFalse(new TRY_STATE(RobotState.NONE));

    conOperator.btn_B
        .onTrue(new TRY_STATE(RobotState.PREP_CORAL_L3))
        .onTrue(new TRY_STATE(RobotState.PREP_CORAL_L3_WITH_ALGAE));

    conOperator.btn_X
        .onTrue(new TRY_STATE(RobotState.PREP_CORAL_L2))
        .onTrue(new TRY_STATE(RobotState.PREP_CORAL_L2_WITH_ALGAE));

    conOperator.btn_Y
        .onTrue(new TRY_STATE(RobotState.PREP_CORAL_L4))
        .onTrue(new TRY_STATE(RobotState.PREP_CORAL_L4_WITH_ALGAE));

    conOperator.btn_LeftStick
        .whileTrue(new TRY_STATE(RobotState.EJECTING))
        .onFalse(new TRY_STATE(RobotState.NONE));

    conOperator.btn_RightStick
        .onTrue(new TRY_STATE(RobotState.PREP_CORAL_ZERO))
        .onTrue(new TRY_STATE(RobotState.PREP_CORAL_ZERO_WITH_ALGAE))
        .onTrue(new TRY_STATE(RobotState.PREP_ALGAE_ZERO));

    conOperator.btn_North
        .onTrue(new TRY_STATE(RobotState.PREP_ALGAE_NET))
        .onTrue(new TRY_STATE(RobotState.PREP_ALGAE_NET_WITH_CORAL));

    conOperator.btn_South
        .onTrue(new TRY_STATE(RobotState.PREP_ALGAE_PROCESSOR))
        .onTrue(new TRY_STATE(RobotState.PREP_ALGAE_PROCESSOR_WITH_CORAL));

    conOperator.btn_East
        .whileTrue(new TRY_STATE(RobotState.CLEAN_HIGH))
        .whileTrue(new TRY_STATE(RobotState.CLEAN_HIGH_WITH_CORAL))
        .onFalse(new TRY_STATE(RobotState.NONE))
        .onFalse(new TRY_STATE(RobotState.HAS_CORAL));

    conOperator.btn_West
        .whileTrue(new TRY_STATE(RobotState.CLEAN_LOW))
        .whileTrue(new TRY_STATE(RobotState.CLEAN_LOW_WITH_CORAL))
        .onFalse(new TRY_STATE(RobotState.NONE))
        .onFalse(new TRY_STATE(RobotState.HAS_CORAL));

    conOperator.btn_Start
        .onTrue(new TRY_STATE(RobotState.HAS_CORAL))
        .onTrue(new TRY_STATE(RobotState.PREP_CORAL_L1));

    conOperator.btn_Back.onTrue(new TRY_STATE(RobotState.HAS_ALGAE));

    hasCoralTrigger.debounce(0.1)
        .whileTrue(new TRY_STATE(RobotState.HAS_CORAL));

    hasAlgaeTrigger
        .whileTrue(new TRY_STATE(RobotState.HAS_ALGAE));

    hasBothTrigger
        .whileTrue(new TRY_STATE(RobotState.HAS_CORAL_AND_ALGAE));

    hasCoralL1Trigger.debounce(0.1)
        .whileTrue(new TRY_STATE(RobotState.PREP_CORAL_L1));

    isCageLatchedTrigger
        .onTrue(new TRY_STATE(RobotState.CLIMBING));
  }

  public boolean allZeroed() {
    return subMotion.hasLiftZeroed && subMotion.hasPivotZeroed && subMotion.hasWristZeroed;
  }

  public RobotState getRobotState() {
    return subStateMachine.getRobotState();
  }

  public Command AddVisionMeasurement() {
    return new AddVisionMeasurement()
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).ignoringDisable(true);
  }
}
