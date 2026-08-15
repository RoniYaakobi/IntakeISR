// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.statemachine.StateMachine;


public class RobotContainer {

  private static RobotContainer instance = null;
  private final StateMachine superStructure;

  public static RobotContainer getInstance(){
    if (instance == null){
      instance = new RobotContainer();
    }
    
    return instance;
  }

  private final CommandXboxController controller;

  private RobotContainer() {
    controller = new CommandXboxController(0);
    superStructure = new StateMachine("SuperStrucure");
    configureSuperStructure();
  }

  private void configureSuperStructure(){

    var autonomous = superStructure.addState(getAutonomousCommand());

    
    superStructure.setInitialState(autonomous);
    
    var isTeleop = new Trigger(() -> Robot.RobotState.getState() == Robot.RobotState.TELEOP);
    
  }

  public Command getSuperStructure(){
    return superStructure;
  }

  private Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
