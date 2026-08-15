// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  public enum RobotState {
    DISABLED,
    AUTONOMOUS,
    TELEOP,
    TEST;

    private static RobotState state;

    public static RobotState getState(){
      return state;
    }

    public static void setState(RobotState state){
      RobotState.state = state;
    }
  }

  private final RobotContainer m_robotContainer;

  public Robot() {
    

    if (RobotBase.isReal()) {
      Logger.addDataReceiver(new NT4Publisher());
      Logger.addDataReceiver(new WPILOGWriter());
    } else {
      Logger.addDataReceiver(new NT4Publisher());

    }

    Logger.start();
    
    m_robotContainer = RobotContainer.getInstance();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    RobotState.setState(RobotState.DISABLED);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    RobotState.setState(RobotState.AUTONOMOUS);
    var superstructure = m_robotContainer.getSuperStructure();

    if (superstructure != null) {
      CommandScheduler.getInstance().schedule(superstructure);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    RobotState.setState(RobotState.TELEOP);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    RobotState.setState(RobotState.TEST);
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
