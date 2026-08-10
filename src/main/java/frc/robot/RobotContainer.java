// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.intakeISR.IntakeISR;


public class RobotContainer {

  private final IntakeISR intake;
  private final SuperStructure superStructure;
  private final CommandXboxController controller;
  public RobotContainer() {
    intake = new IntakeISR();
    superStructure = new SuperStructure(this);
    controller = new CommandXboxController(0);
    configureBindings();
  }

  private void configureBindings() {

  }

  public IntakeISR getIntake(){
    return intake;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
