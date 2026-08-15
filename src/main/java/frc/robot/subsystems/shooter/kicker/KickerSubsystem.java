// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.kicker;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSubsystem extends SubsystemBase {
  /** Creates a new KickerSubsystem. */

  private final KickerIO io;
  private final KickerInputsAutoLogged inputs;

  public KickerSubsystem() {
    io = null;
    inputs = new KickerInputsAutoLogged();
  }

  private void stop(){
    io.stop();
    Logger.recordOutput("KickerSubsystem/DutyCycle", 0);
    Logger.recordOutput("KickerSubsystem/Stopped", true);
  }

  private void setDutyCycle(double dutycycle){
    io.setDutyCycle(dutycycle);
    Logger.recordOutput("KickerSubsystem/DutyCycle", dutycycle);
    Logger.recordOutput("KickerSubsystem/Stopped", false);
  }

  public Command setDutyCycleCommand(double dutycycle){
    return Commands.run(() -> setDutyCycle(dutycycle), this);
  }

  public Command stopCommand(){
    return Commands.run(this::stop, this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }
}
