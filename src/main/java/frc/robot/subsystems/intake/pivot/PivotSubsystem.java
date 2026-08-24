// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.IsNear;

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new Pivot. */
  private final PivotIO io;
  private final PivotInputsAutoLogged inputs;
  
  public PivotSubsystem() {
    io = RobotBase.isReal() ? new PivotIOCTRE() : new PivotIOSim();
    inputs = new PivotInputsAutoLogged();
  }

  /**
   * Manufacture a command that sets the pivot to an angle
   * @param rotation The angle to set the pivot to
   * @return The command that when run sets the pivot to a given angle
   */
  public Command setAngleCommand(Rotation2d rotation){
    return Commands.run(() -> setAngle(rotation), this);
  }

  /**
   * Sets the pivot to an angle
   * @param rotation The angle to set the pivot to
   */
  private void setAngle(Rotation2d rotation){
    io.setAngle(rotation);
    Logger.recordOutput("PivotSubsystem/AngleSetpoint", rotation);
    Logger.recordOutput("PivotSubsystem/stopped", false);
  }

  /**
   * Stops the pivot in place
   */
  private void stop(){
    io.stop();
    Logger.recordOutput("PivotSubsystem/AngleSetpoint", inputs.position);
    Logger.recordOutput("PivotSubsystem/stopped", true);
  }

  /**
   * Manufacture a command that stops the intake pivot
   * @return The command that when run stops the intake pivot.
   */
  public Command stopCommand(){
    return Commands.run(this::stop, this);
  }

  /**
   * Check if the intake is deployed
   * @return Whether or not the intake is deployed
   */
  public boolean isDeployed(){
    return IsNear.isNear(inputs.position, PivotConstants.PIVOT_DEPLOYED_POSITION, PivotConstants.POSITION_TOLERANCE);
  }

  /**
   * Check if the intake is closed
   * @return Whether or not the intake is closed
   */
  public boolean isClosed(){
    return IsNear.isNear(inputs.position, PivotConstants.CLOSE_PIVOT_POSITION, PivotConstants.POSITION_TOLERANCE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }
}
