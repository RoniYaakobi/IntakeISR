// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class RollerSubsystem extends SubsystemBase {
  /** Creates a new Roller. */
  private final RollerIO io;
  private final RollerInputsAutoLogged inputs;
  
  public RollerSubsystem() {
    io = RobotBase.isReal() ? new RollerIOCTRE() : new RollerIOSim();
    inputs = new RollerInputsAutoLogged();

  }

  /**
   * Manufacture a command which applies a dutycycle output 
   * @param dutyCycle The dutycycle output
   * @return A command that when run applies a dutycycle
   */
  public Command setDutyCycleCommand(double dutyCycle){
    return Commands.run(() -> setDutyCycle(dutyCycle), this);
  }

  /**
   * Sets a dutycycle ouput for the intake roller
   * @param dutyCycle The dutycycle to apply
   */
  private void setDutyCycle(double dutyCycle){
    io.setDutyCycle(dutyCycle);
    Logger.recordOutput("RollerSubsystem/dutycycle", dutyCycle);
    Logger.recordOutput("RollerSubsystem/Stopped", false);

  }

  /**
   * Manufacture a command that when run stops the intake roller
   * @return A command that when run stops the intake roller
   */
  public Command stopCommand(){
    return Commands.run(this::stop, this);
  }

  /**
   * Stop the intake roller
   */
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
