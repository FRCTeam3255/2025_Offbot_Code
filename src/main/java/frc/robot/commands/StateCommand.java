// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

/**
 * Abstract base class for state machine commands that validates state
 * transitions.
 * 
 * Each StateCommand defines the states it can transition from and automatically
 * validates the transition when scheduled. If the transition is invalid, an
 * InstantCommand is scheduled to print an error message instead.
 * 
 * Users override initialize() and execute() like normal WPILib commands.
 */
public abstract class StateCommand extends Command {

  /**
   * Returns the set of states this command can transition from.
   * 
   * @return Set of valid previous states
   */
  protected abstract Set<Class<? extends StateCommand>> getAllowedPreviousStates();

  public StateCommand() {
  }

  /**
   * Validates if the state transition is legal.
   */
  private boolean isValidTransition() {
    Class<? extends StateCommand> currentState = RobotContainer.getRobotState();
    Set<Class<? extends StateCommand>> allowedStates = getAllowedPreviousStates();
    return allowedStates.contains(currentState);
  }

  @Override
  public final void schedule() {
    if (!isValidTransition()) {
      Commands.print("ERROR: Invalid state transition. Attempted to go from " +
          RobotContainer.getRobotState() + " to " + this.getClass() +
          ". Allowed previous states: " + getAllowedPreviousStates()).schedule();
      return;
    }

    RobotContainer.setRobotState(this.getClass());
    super.schedule();
  }

  @Override
  public boolean isFinished() {
    return false; // Default: runs until interrupted
  }
}
