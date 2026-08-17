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

  public Command setAngleCommand(Rotation2d rotation){
    return Commands.run(() -> setAngle(rotation), this);
  }

  private void setAngle(Rotation2d rotation){
    io.setAngle(rotation);
    Logger.recordOutput("HoodSubsystem/angleSetpoint", rotation);
    Logger.recordOutput("HoodSubsystem/stopped", false);
  }

  private void stop(){
    io.stop();
    Logger.recordOutput("HoodSubsystem/angleSetpoint", Rotation2d.kZero);
    Logger.recordOutput("HoodSubsystem/stopped", true);
  }

  public Command stopCommand(){
    return Commands.run(this::stop, this);
  }

  public boolean waitUntilClose(Rotation2d goal){
    return IsNear.isNear(inputs.angle, goal, ShooterConstants.HOOD_TOLERANCE);
  }

  public Command closeHoodCommand(){
    var closeHood = new StateMachine("close hood");

    var zeroHood = closeHood.addState(setAngleCommand(Rotation2d.kZero), ShooterConstants.ZERO_HOOD);

    var turnOffHood = closeHood.addState(closeHoodCommand(), ShooterConstants.CLOSE_HOOD);

    closeHood.setInitialState(zeroHood);
    zeroHood.switchTo(turnOffHood).when(() -> waitUntilClose(Rotation2d.kZero));

    return closeHood;

  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }
}
