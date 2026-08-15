// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakeISR;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeISR extends SubsystemBase {

  public final IntakeISRIO io;
  public final IntakeISRInputsAutoLogged inputs;


  /** Creates a new IntakeISR. */
  public IntakeISR() {
    io = new IntakeISRIORev();
    inputs = new IntakeISRInputsAutoLogged();
  }

  public Command activate(){
    return new FunctionalCommand(
      () -> io.goToRotation(IntakeISRConstants.OPEN_ROTATION), () -> {},
      (interrupted) -> io.setRollerSpeed(0),
      this::isPivotAtGoal , this);
  }

  public Command close(){
    return Commands.runOnce(
    () -> {
      io.stopRoller();
      io.goToRotation(IntakeISRConstants.CLOSED_ROTATION);
    } , this);
  }

  private boolean isPivotAtGoal(){
    return inputs.positionRotation.minus(IntakeISRConstants.OPEN_ROTATION).getDegrees() 
            <= IntakeISRConstants.OPEN_TOLERANCE.getDegrees();
  }


  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }
}
