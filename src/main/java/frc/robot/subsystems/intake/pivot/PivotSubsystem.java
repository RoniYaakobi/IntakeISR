// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.math.IsNear;

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new Pivot. */
  private final PivotIO io;
  private final PivotInputsAutoLogged inputs;
  
  public PivotSubsystem() {
    io = null;
    inputs = new PivotInputsAutoLogged();
  }

  public Command setAngleCommand(Rotation2d rotation){
    return Commands.run(() -> setAngle(rotation), this);
  }

  private void setAngle(Rotation2d rotation){
    io.setAngle(rotation);
    Logger.recordOutput("PivotSubsystem/AngleSetpoint", rotation);
    Logger.recordOutput("PivotSubsystem/stopped", false);
  }

  private void stop(){
    io.stop();
    Logger.recordOutput("PivotSubsystem/AngleSetpoint", inputs.position);
    Logger.recordOutput("PivotSubsystem/stopped", true);
  }

  public Command stopCommand(){
    return Commands.run(this::stop, this);
  }

  public Command waitUntilOpen(){
    return new WaitUntilCommand(
      () -> IsNear.isNear(inputs.position, PivotConstants.OPEN_PIVOT_POSITION, PivotConstants.POSITION_TOLERANCE)
    );
  }

  public Command waitUntilClose(){
    return new WaitUntilCommand(
      () -> IsNear.isNear(inputs.position, PivotConstants.CLOSE_PIVOT_POSITION, PivotConstants.POSITION_TOLERANCE)
    );
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }
}
