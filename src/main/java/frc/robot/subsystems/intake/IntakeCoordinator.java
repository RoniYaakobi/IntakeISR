// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.statemachine.StateMachine;
import frc.robot.subsystems.intake.pivot.PivotConstants;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.intake.roller.RollerConstants;
import frc.robot.subsystems.intake.roller.RollerSubsystem;

public class IntakeCoordinator {
  /** Creates a new IntakeSubsystem. */
  private final PivotSubsystem pivot;
  private final RollerSubsystem roller;

  public IntakeCoordinator() {
    pivot = new PivotSubsystem();
    roller = new RollerSubsystem();
  }

  public Command activateIntake(){
    var activateIntake = new StateMachine("Activate Intake");

    var openIntake = activateIntake.addState(
      pivot.setAngleCommand(PivotConstants.OPEN_PIVOT_POSITION), 
      IntakeConstants.OPEN_INTAKE_STATE_NAME);

    var closeIntake = activateIntake.addState(
      roller.setDutyCycleCommand(RollerConstants.ACTIVE_DUTYCYCLE), 
      IntakeConstants.CLOSE_INTAKE_STATE_NAME);

    activateIntake.setInitialState(openIntake);
    openIntake.switchTo(closeIntake).when(pivot::isOpen);

    return activateIntake;
  }

  public Command disableIntake(){
    var disableIntake = new StateMachine("DISABLE_INTAKE");

    var closeIntake = disableIntake.addState(Commands.parallel(
        roller.stopCommand(),
        pivot.setAngleCommand(PivotConstants.CLOSE_PIVOT_POSITION)
      ), IntakeConstants.CLOSE_INTAKE_STATE_NAME);

    var haltPivot = disableIntake.addState(pivot.stopCommand(), IntakeConstants.HALT_INTAKE_STATE_NAME);

    disableIntake.setInitialState(closeIntake);
    closeIntake.switchTo(haltPivot).when(pivot::isClosed);

    return disableIntake;
  }
}
