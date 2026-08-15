// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.pivot.PivotConstants;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.intake.roller.RollerConstants;
import frc.robot.subsystems.intake.roller.RollerSubsystem;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final PivotSubsystem pivot;
  private final RollerSubsystem roller;

  public IntakeSubsystem() {
    pivot = new PivotSubsystem();
    roller = new RollerSubsystem();
  }

  public Command openIntake(){
    return Commands.sequence(
      pivot.setAngleCommand(PivotConstants.OPEN_PIVOT_POSITION),
      pivot.waitUntilOpen(),
      roller.setDutyCycleCommand(RollerConstants.ACTIVE_DUTYCYCLE)
    );
  }

  public Command closeIntake(){
    return Commands.sequence(
      roller.stopCommand(),
      pivot.setAngleCommand(PivotConstants.CLOSE_PIVOT_POSITION),
      pivot.waitUntilClose(),
      pivot.stopCommand()
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
