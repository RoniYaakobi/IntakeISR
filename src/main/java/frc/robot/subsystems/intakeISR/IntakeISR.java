// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakeISR;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeISR extends SubsystemBase {

  // The state I want to hold
  public enum DesiredState{
    ACTIVE, // intake deployed and roller on
    CLOSED // intake closed and roller off
  }

  // The state I currently am in
  public enum CurrentState {
    ACTIVATING, // deploy intake and turn on roller
    CLOSING // close intake and turn off roller
  }

  public final IntakeISRIO io;
  public final IntakeISRInputsAutoLogged inputs;
  public DesiredState desiredState;
  public CurrentState currentState;


  /** Creates a new IntakeISR. */
  public IntakeISR() {
    desiredState = DesiredState.CLOSED;
    currentState = CurrentState.CLOSING;

    io = new IntakeISRIORev();
    inputs = new IntakeISRInputsAutoLogged();
  }

  // Transition based on the desired state to the current state. Enforce rules here
  private CurrentState handleStateTransition(){
    return switch (desiredState){
      case ACTIVE -> CurrentState.ACTIVATING;
      case CLOSED -> CurrentState.CLOSING;
    };
  }

  // Run the current state
  private void applyState(){
    switch (currentState){
      case ACTIVATING -> {
        io.goToRotation(IntakeISRConstants.OPEN_ROTATION);
        if (inputs.positionRotation.minus(IntakeISRConstants.OPEN_ROTATION).getDegrees() 
            <= IntakeISRConstants.OPEN_TOLERANCE.getDegrees()){
          io.setRollerSpeed(IntakeISRConstants.ROLLER_SPEED_RPS);
        }
      }
      case CLOSING -> {
        io.stopRoller();
        io.goToRotation(IntakeISRConstants.CLOSED_ROTATION);
      }
    };
  }

  // Attempt to trigger a state 
  public void setDesiredState(DesiredState desiredState){
    this.desiredState = desiredState;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    currentState = handleStateTransition();
    Logger.recordOutput(getName() + "/Desired State", desiredState);
    Logger.recordOutput(getName() + "/Current State", currentState);
    applyState();
  }
}
