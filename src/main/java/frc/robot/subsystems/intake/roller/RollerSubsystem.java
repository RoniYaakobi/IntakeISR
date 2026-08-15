// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {
  /** Creates a new Roller. */
  private final RollerIO io;
  private final RollerInputsAutoLogged inputs;
  
  public RollerSubsystem() {
    io = null;
    inputs = new RollerInputsAutoLogged();

  }

  public Command setDutyCycleCommand(double dutyCycle){
    return Commands.run(() -> setDutyCycle(dutyCycle), this);
  }

  private void setDutyCycle(double dutyCycle){
    io.setDutyCycle(dutyCycle);
    Logger.recordOutput("RollerSubsystem/dutycycle", dutyCycle);
    Logger.recordOutput("RollerSubsystem/Stopped", false);

  }

  public Command stopCommand(){
    return Commands.run(this::stop, this);
  }

  private void stop(){
    io.stop();
    Logger.recordOutput("RollerSubsystem/dutycycle", 0);
    Logger.recordOutput("RollerSubsystem/Stopped", true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }
}
