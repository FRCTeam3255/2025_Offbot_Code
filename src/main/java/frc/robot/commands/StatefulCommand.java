// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Objects;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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
public abstract class StatefulCommand extends Command {

  private static Class<? extends StatefulCommand> currentState;

  /**
   * Gets the current robot state.
   * 
   * @return The current state command class
   */
  public static Class<? extends StatefulCommand> getCurrentState() {
    return currentState;
  }

  /**
   * Sets the robot state manually.
   * 
   * @param state The new state
   */
  public static void setState(Class<? extends StatefulCommand> state) {
    currentState = Objects.requireNonNull(state, "Robot state cannot be null");
  }

  /**
   * Returns the set of states this command can transition from.
   * 
   * @return Set of valid previous states
   */
  protected abstract Set<Class<? extends StatefulCommand>> getAllowedPreviousStates();

  public StatefulCommand() {
  }

  /**
   * Validates if the state transition is legal.
   */
  private boolean isValidTransition() {
    Set<Class<? extends StatefulCommand>> allowedStates = getAllowedPreviousStates();
    return allowedStates.contains(currentState);
  }

  @Override
  public final void schedule() {
    if (!isValidTransition()) {
      Commands.print("ERROR: Invalid state transition. Attempted to go from " +
          currentState.getSimpleName() + " to " + this.getClass().getSimpleName() +
          ". Allowed previous states: " + getAllowedPreviousStates()).schedule();
      return;
    }

    setState(this.getClass());
    super.schedule();
  }

  @Override
  public boolean isFinished() {
    return false; // Default: runs until interrupted
  }
}
