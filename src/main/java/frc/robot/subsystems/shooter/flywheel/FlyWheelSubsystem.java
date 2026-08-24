// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.IsNear;
import frc.robot.subsystems.shooter.ShooterConstants;

public class FlyWheelSubsystem extends SubsystemBase {
  /** Creates a new FlyWheelSubsystem. */
  private final FlywheelIO io;
  private final FlyWheelInputsAutoLogged inputs;
  
  public FlyWheelSubsystem() {
    io = RobotBase.isReal() ? new FlyWheelIORev() : new FlyWheelIOSim();
    inputs = new FlyWheelInputsAutoLogged();
  }

  /**
   * Manufactures a command which sets the flywheel speed to a given speed
   * @param speedMPS The speed the manufactured command needs to set
   * @return The command that when run sets the flywheel speed
   */
  public Command setSpeedMPSCommand(double speedMPS){
    return Commands.run(() -> setSpeedMPS(speedMPS), this);
  }

  /**
   * Sets the flywheel speed in mps
   * @param speedMPS The goal speed in MPS
   */
  private void setSpeedMPS(double speedMPS){
    io.setSpeedMPS(speedMPS);
    Logger.recordOutput("FlywheelSubsystem/FlywheelSetpointMPS", speedMPS);
    Logger.recordOutput("FlywheelSubsystem/Stopped", false);
  }

  /**
   * Stops the flywheel
   */
  private void stop(){
    io.stop();
    Logger.recordOutput("FlywheelSubsystem/FlywheelSetpointMPS", 0);
    Logger.recordOutput("FlywheelSubsystem/Stopped", true);
  }

  /**
   * Is the motor spun up to a speed
   * @param speedMPS The speed that needs to be reached
   * @return Whether or not the speed has been reached
   */
  public boolean isSpunUp(double speedMPS){
    return IsNear.isNear(speedMPS, inputs.speedMPS, ShooterConstants.FLYWHEEL_MPS_TOLERANCE);
  }

  /**
   * Manufacture a command which stops the flywheel 
   * @return A command that when run stops the flywheel
   */
  public Command stopCommand(){
    return Commands.run(this::stop, this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }
}
