// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotState;

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
  protected abstract Set<RobotState> getAllowedPreviousStates();

  /**
   * Returns the desired state this command transitions to.
   * 
   * @return The target robot state
   */
  protected abstract RobotState getDesiredState();

  public StateCommand() {
  }

  /**
   * Validates if the state transition is legal.
   */
  private boolean isValidTransition() {
    RobotState currentState = RobotContainer.getRobotState();
    Set<RobotState> allowedStates = getAllowedPreviousStates();
    return allowedStates.contains(currentState);
  }

  @Override
  public final void schedule() {
    if (!isValidTransition()) {
      Commands.print("ERROR: Invalid state transition. Attempted to go from " +
          RobotContainer.getRobotState() + " to " + getDesiredState() +
          ". Allowed previous states: " + getAllowedPreviousStates()).schedule();
      return;
    }

    RobotContainer.setRobotState(getDesiredState());
    super.schedule();
  }

  @Override
  public boolean isFinished() {
    return false; // Default: runs until interrupted
  }
}
