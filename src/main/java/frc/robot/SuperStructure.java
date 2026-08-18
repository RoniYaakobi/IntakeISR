// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMachine.State;
import frc.robot.subsystems.indexer.TwindexerConstants;
import frc.robot.subsystems.indexer.TwindexerSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeCoordinator;
import frc.robot.subsystems.shooter.ShooterConstants.ShootParams;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterCoordinator;

public class SuperStructure extends SubsystemBase {
    /** Creates a new SuperStructure. */
    public static final Trigger IS_TELEOP = new Trigger(RobotState::isTeleop);
    
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;

    private final StateMachine statemachine;

    private final ShooterCoordinator shooter;
    private final IntakeCoordinator intake;
    private final TwindexerSubsystem twindexer;

    private static SuperStructure superstructure = null;

    public static SuperStructure getInstance(){
        if (superstructure == null){
        superstructure = new SuperStructure();
        }

        return superstructure;
    }

    private SuperStructure() {
        statemachine = new StateMachine("SuperStructure");
        driverController = RobotContainer.getInstance().getDriverController();
        operatorController = RobotContainer.getInstance().getOperatorController();

        shooter = RobotContainer.getInstance().getShooter();
        intake = RobotContainer.getInstance().getIntake();
        twindexer = RobotContainer.getInstance().getTwindexer();

        configureBindings();
        configureTransitions();
    }

    private void configureBinding(State state, Trigger binding){
        statemachine.switchFromAny().to(state).when(binding);
    }

    private void configureBindings(){
        var activateIntake = statemachine.addState(intake.activateIntake(), IntakeConstants.INTAKE_ACTIVATE_STATE_NAME);
        configureBinding(activateIntake, driverController.b());

        var closeIntake = statemachine.addState(intake.disableIntake(), IntakeConstants.INTAKE_DISABLE_STATE_NAME);
        configureBinding(closeIntake, driverController.a());

        var spinUpAndShoot = statemachine.addState(
            shooter.spinUpAndShootCommand(
                new ShootParams(0, 0, Rotation2d.kZero), 
                new ShootParams(0, 0, Rotation2d.kZero)),
            ShooterConstants.SPIN_UP_AND_SHOOT_STATE_NAME);
        configureBinding(spinUpAndShoot, driverController.x());

        var stopShooting = statemachine.addState(shooter.stopCommand(), ShooterConstants.STOP_SHOOTING_STATE_NAME);
        configureBinding(stopShooting, driverController.y());

        var index = statemachine.addState(twindexer.indexCommand(0), TwindexerConstants.INDEX_STATE_NAME);
        configureBinding(index, driverController.leftBumper());

        var stopIndexing = statemachine.addState(twindexer.stopCommand(), TwindexerConstants.STOP_INDEXING_NAME);
        configureBinding(stopIndexing, driverController.rightBumper());
    }

    private void configureTransitions(){
        var autonomous = statemachine.addState(
            RobotContainer.getInstance().getAutonomousCommand(),
            Constants.AUTONOMOUS_STATE_NAME
        );

        statemachine.setInitialState(autonomous);
    }

    public Command getCommand(){
        return statemachine;
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
