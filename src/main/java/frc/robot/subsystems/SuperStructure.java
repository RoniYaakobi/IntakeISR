// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class SuperStructure extends SubsystemBase {
  /** Creates a new SuperStructure. */

    // The states that the superstructure can attempt to be in
    public enum DesiredState{
    INTAKE, // Intaking balls
    CLOSED // All subsystems are closed
  }

  // The states that the superstructure can be in
  public enum CurrentState {
    INTAKING, // Open intake roller on
    CLOSING // Subsystems are closed
  }

  public DesiredState desiredState;
  public CurrentState currentState;

  // Instead of passing all of the subsystems explicitly, pass the robotContainer with getters for all the subsystems.
  private final RobotContainer robotContainer;

  public SuperStructure(RobotContainer robotContainer) {
    desiredState = DesiredState.CLOSED;
    currentState = CurrentState.CLOSING;
    this.robotContainer = robotContainer;
  }

  // Transition logic (enforce rules here)
  private CurrentState handleStateTransition(){
    return switch (desiredState){
      case INTAKE -> CurrentState.INTAKING;
      case CLOSED -> CurrentState.CLOSING;
    };
  }

  // Run the state's logic
  private void applyState(){
    switch (currentState){
      case INTAKING -> {
        robotContainer.getIntake().setDesiredState(frc.robot.subsystems.intakeISR.IntakeISR.DesiredState.ACTIVE);
      }
      case CLOSING -> {
        robotContainer.getIntake().setDesiredState(frc.robot.subsystems.intakeISR.IntakeISR.DesiredState.CLOSED);
      }
    };
  }

  // Attempt to trigger the superstructure desired state to be something
  public void setDesiredState(DesiredState desiredState){
    this.desiredState = desiredState;
  }

  // Transition to Intake mode
  public void startIntake(){
    setDesiredState(DesiredState.INTAKE);
  }

  // Transition to stop intake mode
  public void stopIntake(){
    setDesiredState(DesiredState.CLOSED);
  }


  @Override
  public void periodic() {
    currentState = handleStateTransition();
    Logger.recordOutput(getName() + "/Desired State", desiredState);
    Logger.recordOutput(getName() + "/Current State", currentState);
    applyState();
  }
}
