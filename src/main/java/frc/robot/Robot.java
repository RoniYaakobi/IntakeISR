// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.shooter.kicker.KickerSubsystem;

public class Robot extends LoggedRobot {

    KickerSubsystem s;
    CommandXboxController x;
  public Robot() {
    

    if (RobotBase.isReal()) {
      Logger.addDataReceiver(new NT4Publisher());
      Logger.addDataReceiver(new WPILOGWriter());
    } else {
      Logger.addDataReceiver(new NT4Publisher());

    }

    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {    
    // var superstructure = SuperStructure.getInstance().getCommand();

    // if (superstructure != null) {
    //   CommandScheduler.getInstance().schedule(superstructure);
    // }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    x= new CommandXboxController(0);
    s = new KickerSubsystem();

    x.a().onTrue(s.setDutyCycleCommand(0.2));
    x.b().onTrue(s.setDutyCycleCommand(0.5));
    x.y().onTrue(s.stopCommand());
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
