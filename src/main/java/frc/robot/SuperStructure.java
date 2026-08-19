// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMachine.State;
import frc.lib.statemachine.StateMachine.StateName;
import frc.robot.subsystems.indexer.TwindexerConstants;
import frc.robot.subsystems.indexer.TwindexerSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeCoordinator;
import frc.robot.subsystems.shooter.ShooterConstants.ShootParams;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterCoordinator;

public class SuperStructure extends SubsystemBase {
    /** Creates a new SuperStructure. */
    private final CommandXboxController driverController;
    private final List<State> states;


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

        states = new ArrayList<>();

        shooter = RobotContainer.getInstance().getShooter();
        intake = RobotContainer.getInstance().getIntake();
        twindexer = RobotContainer.getInstance().getTwindexer();

        statemachine.setInitialState(
            registerState(
                this.idle(), 
                Constants.IDLE_STATE_NAME
            )
        );

        configureBindings();
    }

    private State registerState(Command cmd, StateName stateName){
        var state = statemachine.addState(cmd, stateName);
        states.add(state);
        return state;
    }

    private void switchFromAnyOtherThanMyself(State to, Trigger trigger){
        for (State from : states){
            if (from == to) {
                continue;
            }

            from.switchTo(to).when(trigger);
        }
    }

    private void configureBinding(State to, Trigger binding){
        switchFromAnyOtherThanMyself(to, binding);
    }

    private void configureBindings(){
        var activateIntake = 
                registerState(
                    intake.activateIntake().withName("b").repeatedly(), 
                    IntakeConstants.INTAKE_ACTIVATE_STATE_NAME);

        var closeIntake = 
                registerState(
                    intake.disableIntake().withName("a").repeatedly(), 
                    IntakeConstants.INTAKE_DISABLE_STATE_NAME);
        
        var spinUpAndShoot = 
                registerState(
                    shooter.spinUpAndShootCommand(
                        new ShootParams(10, 0, Rotation2d.fromRotations(0.04)), 
                        new ShootParams(15, 0.6, Rotation2d.fromRotations(0.08333))
                    ).withName("x").repeatedly(),
                    ShooterConstants.SPIN_UP_AND_SHOOT_STATE_NAME);
        
        var stopShooting = 
                registerState(
                    shooter.stopCommand().withName("y").repeatedly(), 
                    ShooterConstants.STOP_SHOOTING_STATE_NAME);
        
        var index = 
                registerState(
                    twindexer.indexCommand(0).withName("left bumper"), 
                    TwindexerConstants.INDEX_STATE_NAME);

        var stopIndexing = 
                registerState(
                    twindexer.stopCommand().withName("right bumper"), 
                    TwindexerConstants.STOP_INDEXING_NAME);

        configureBinding(index, driverController.leftBumper());
        configureBinding(stopShooting, driverController.y());
        configureBinding(spinUpAndShoot, driverController.x());
        configureBinding(stopIndexing, driverController.rightBumper());
        configureBinding(activateIntake, driverController.b());
        configureBinding(closeIntake, driverController.a());
    }

    public Command getCommand(){
        return statemachine;
    }
}
