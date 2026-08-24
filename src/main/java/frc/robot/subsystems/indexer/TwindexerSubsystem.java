// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TwindexerSubsystem extends SubsystemBase {
  /** Creates a new Twindexer. */
  private final TwindexerIO io;
  private final TwindexerInputsAutoLogged inputs;

  public TwindexerSubsystem() {
    io = RobotBase.isReal() ? new TwindexerIORev() : new TwindexerIOSim();
    inputs = new TwindexerInputsAutoLogged();

  }

  /**
   * Manufacture a command that activates the indexer
   * @param dutyCycle The dutycycle to apply to the indexer
   * @return A command that when run activates the indexer
   */
  public Command indexCommand(double dutyCycle){
    return Commands.run(() -> setDutyCycle(dutyCycle), this);
  }

  /**
   * Manufacture a command that stops the indexer
   * @return A command that when run stops the indexer
   */
  public Command stopCommand(){
    return Commands.run(this::stop, this);
  }

  /**
   * Apply a given dutycycle to the indexer
   * @param dutyCycle The dutycycle to apply
   */
  private void setDutyCycle(double dutyCycle){
    io.setDutyCycle(dutyCycle);
    Logger.recordOutput("Twindexer/dutyCycle", dutyCycle);
    Logger.recordOutput("Twindexer/stopped", false);
  }

  /**
   * Stop the indexer
   */
  private void stop(){
    io.stop();
    Logger.recordOutput("Twindexer/dutyCycle", 0);
    Logger.recordOutput("Twindexer/stopped", true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }
}
