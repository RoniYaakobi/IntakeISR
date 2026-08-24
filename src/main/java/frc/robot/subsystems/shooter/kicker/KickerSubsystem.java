// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.kicker;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSubsystem extends SubsystemBase {
  /** Creates a new KickerSubsystem. */

  private final KickerIO io;
  private final KickerInputsAutoLogged inputs;

  public KickerSubsystem() {
    io = RobotBase.isReal() ? new KickerIORev() : new KickerIOSim();
    inputs = new KickerInputsAutoLogged();
  }

  /**
   * Stop the kicker.
   */
  private void stop(){
    io.stop();
    Logger.recordOutput("KickerSubsystem/DutyCycle", 0);
    Logger.recordOutput("KickerSubsystem/Stopped", true);
  }

  /**
   * Applies a dutycycle output to the kicker.
   * @param dutycycle The dutycycle to apply to the kicker.
   */
  private void setDutyCycle(double dutycycle){
    io.setDutyCycle(dutycycle);
    Logger.recordOutput("KickerSubsystem/DutyCycle", dutycycle);
    Logger.recordOutput("KickerSubsystem/Stopped", false);
  }

  /**
   * Returns command that when run sets a given duty cycle to the motor. 
   * @param dutycycle The dutycycle to apply.
   * @return The command to run to apply the dutycycle.
   */
  public Command setDutyCycleCommand(double dutycycle){
    return Commands.run(() -> setDutyCycle(dutycycle), this);
  }

  /**
   * Returns a command that when run stops the kicker.
   * @return The command to run to stop the kicker.
   */
  public Command stopCommand(){
    return Commands.run(this::stop, this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    var cmd = getCurrentCommand();
    Logger.recordOutput(getName() + "/Command", cmd == null ? "None" : cmd.getName());
  }
}
