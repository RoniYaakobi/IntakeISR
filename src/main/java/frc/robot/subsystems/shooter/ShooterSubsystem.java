// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.statemachine.StateMachine;
import frc.robot.subsystems.shooter.ShooterConstants.ShootParams;
import frc.robot.subsystems.shooter.flywheel.FlyWheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.kicker.KickerSubsystem;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final FlyWheelSubsystem flywheel;
  private final HoodSubsystem hood;
  private final KickerSubsystem kicker;

  public ShooterSubsystem() {
    flywheel = new FlyWheelSubsystem();
    hood = new HoodSubsystem();
    kicker = new KickerSubsystem();
  }

  public Command shootCommand(ShootParams params){
    return Commands.parallel(
      flywheel.setSpeedMPSCommand(params.flywheelspeedMPS()),
      kicker.setDutyCycleCommand(params.dutycycleKicker()),
      hood.setAngleCommand(params.hoodAngle())
    );
  }

  public Command spinUpCommand(ShootParams params){
    return Commands.parallel(
      flywheel.setSpeedMPSCommand(params.flywheelspeedMPS()),
      hood.setAngleCommand(params.hoodAngle()));
  }

  public Command spinUpAndShootCommand(ShootParams spinUpParams, ShootParams shootParams){
    var spinUpAndShoot = new StateMachine("SpinUpAndShoot");
    
    var spinUp = spinUpAndShoot.addState(spinUpCommand(spinUpParams), ShooterConstants.SPIN_UP);

    var shoot = spinUpAndShoot.addState(shootCommand(shootParams), ShooterConstants.SHOOT);
    
    spinUpAndShoot.setInitialState(spinUp);
    spinUp.switchTo(shoot).when(() -> flywheel.isSpunUp(0));
    
    return spinUpAndShoot;
  }

  public Command stopCommand(){
    return Commands.parallel(
      flywheel.stopCommand(),
      hood.closeHoodCommand(),
      kicker.stopCommand()
    );
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
