// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.statemachine.StateMachine;
import frc.robot.subsystems.shooter.ShooterConstants.ShootParams;
import frc.robot.subsystems.shooter.flywheel.FlyWheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.kicker.KickerSubsystem;

public class ShooterCoordinator {
  /** Creates a new ShooterSubsystem. */
  private final FlyWheelSubsystem flywheel;
  private final HoodSubsystem hood;
  private final KickerSubsystem kicker;

  public ShooterCoordinator() {
    flywheel = new FlyWheelSubsystem();
    hood = new HoodSubsystem();
    kicker = new KickerSubsystem();
  }

  /**
   * A command that makes your robot shoot.
   * @param params The shooting parameters.
   * @return A command to shoot that can be executed.
   */
  public Command shootCommand(ShootParams params){
    return Commands.parallel(
      flywheel.setSpeedMPSCommand(params.flywheelspeedMPS()),
      kicker.setDutyCycleCommand(params.dutycycleKicker()),
      hood.setAngleCommand(params.hoodAngle())
    );
  }

  /**
   * A command that makes your robot spin up the shooter.
   * @param params The spin up parameters.
   * @return A command to spin up that can be executed.
   */
  public Command spinUpCommand(ShootParams params){
    return Commands.parallel(
      flywheel.setSpeedMPSCommand(params.flywheelspeedMPS()),
      hood.setAngleCommand(params.hoodAngle()));
  }

  /**
   * A command (statemachine) that makes you first spin up and then shoot. 
   * @param spinUpParams The parameters for spin up.
   * @param shootParams The parameters for shooting.
   * @return The command to run spin up and shoot.
   */
  public Command spinUpAndShootCommand(ShootParams spinUpParams, ShootParams shootParams){
    var spinUpAndShoot = new StateMachine("SpinUpAndShoot");
    
    var spinUp = spinUpAndShoot.addState(spinUpCommand(spinUpParams), ShooterConstants.SPIN_UP);

    var shoot = spinUpAndShoot.addState(shootCommand(shootParams), ShooterConstants.SHOOT);
    
    spinUpAndShoot.setInitialState(spinUp);
    spinUp.switchTo(shoot).when(() -> flywheel.isSpunUp(0));
    
    return spinUpAndShoot;
  }

  /**
   * Stop the shooter subsystems.
   * @return The command to stop the shooter subsystems.
   */
  public Command stopCommand(){
    return Commands.parallel(
      flywheel.stopCommand(),
      hood.closeHoodCommand(),
      kicker.stopCommand()
    );
  }
}
