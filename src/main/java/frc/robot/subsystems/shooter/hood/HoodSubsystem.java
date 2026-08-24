// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.IsNear;
import frc.lib.statemachine.StateMachine;
import frc.robot.subsystems.shooter.ShooterConstants;

public class HoodSubsystem extends SubsystemBase {
  /** Creates a new HoodSubsystem. */
  private final HoodIO io;
  private final HoodInputsAutoLogged inputs;


  public HoodSubsystem() {
    io = RobotBase.isReal() ? new HoodIORev() : new HoodIOSim();
    inputs = new HoodInputsAutoLogged();
  }

  /**
   * A factory for commands that set the hood angle
   * @param rotation The angle the hood needs to go to
   * @return A command that when run gives take the hood to an angle.
   */
  public Command setAngleCommand(Rotation2d rotation){
    return Commands.run(() -> setAngle(rotation), this);
  }

  /**
   * Tell the hood to go to a certain angle
   * @param rotation The rotation the hood needs to go to.
   */
  private void setAngle(Rotation2d rotation){
    io.setAngle(rotation);
    Logger.recordOutput("HoodSubsystem/angleSetpoint", rotation);
    Logger.recordOutput("HoodSubsystem/stopped", false);
  }

  /**
   * Stop the hood motor.
   */
  private void stop(){
    io.stop();
    Logger.recordOutput("HoodSubsystem/angleSetpoint", Rotation2d.kZero);
    Logger.recordOutput("HoodSubsystem/stopped", true);
  }

  /**
   * Command that stops the hood
   * @return A command that when run stops the hood.
   */
  public Command stopCommand(){
    return Commands.run(this::stop, this);
  }


  /**
   * Checks whether the hood is at an angle
   * @param goal The angle that needs to be reached
   * @return Whether or not the angle has been reached
   */
  public boolean isAtAngle(Rotation2d goal){
    return IsNear.isNear(inputs.angle, goal, ShooterConstants.HOOD_TOLERANCE);
  }

  /**
   * Factory method that returns a command which closes the hood
   * @return A command that closes the hood.
   */
  public Command closeHoodCommand(){
    var closeHood = new StateMachine("close hood");

    var zeroHood = closeHood.addState(setAngleCommand(Rotation2d.kZero), HoodConstants.ZERO_HOOD);

    var turnOffHood = closeHood.addState(stopCommand(), HoodConstants.TURN_OFF);

    closeHood.setInitialState(zeroHood);
    zeroHood.switchTo(turnOffHood).when(() -> isAtAngle(Rotation2d.kZero));

    return closeHood;

  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }
}
