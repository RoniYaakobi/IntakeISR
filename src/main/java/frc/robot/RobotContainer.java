// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.indexer.TwindexerSubsystem;
import frc.robot.subsystems.intake.IntakeCoordinator;
import frc.robot.subsystems.shooter.ShooterCoordinator;


public class RobotContainer {

  private static RobotContainer instance = null;
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;

  private final ShooterCoordinator shooter;
  private final IntakeCoordinator intake;
  private final TwindexerSubsystem twindexer;


  public static RobotContainer getInstance(){
    if (instance == null){
      instance = new RobotContainer();
    }
    
    return instance;
  }

  private RobotContainer() {
    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    shooter = new ShooterCoordinator();
    intake = new IntakeCoordinator();
    twindexer = new TwindexerSubsystem();

  }

  public CommandXboxController getDriverController(){
    return driverController;
  }

  public CommandXboxController getOperatorController(){
    return operatorController;
  }

  public ShooterCoordinator getShooter(){
    return shooter;
  }

  public IntakeCoordinator getIntake(){
    return intake;
  }

  public TwindexerSubsystem getTwindexer(){
    return twindexer;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
