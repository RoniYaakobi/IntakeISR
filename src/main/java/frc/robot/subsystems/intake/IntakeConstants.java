package frc.robot.subsystems.intake;

import frc.lib.statemachine.StateMachine.StateName;

public class IntakeConstants {
    public final static StateName INTAKE_ACTIVATE_STATE_NAME = new StateName("ACTIVATE_INTAKE");
    public final static StateName INTAKE_DISABLE_STATE_NAME = new StateName("DISABLE_INTAKE");

    public final static StateName OPEN_INTAKE_STATE_NAME = new StateName("OPEN_INTAKE");
    public final static StateName START_INTAKE_ROLLER_STATE_NAME = new StateName("START_INTAKE_ROLLER");

    public final static StateName CLOSE_INTAKE_STATE_NAME = new StateName("CLOSE_INTAKE");
    public final static StateName HALT_INTAKE_STATE_NAME = new StateName("HALT_INTAKE");
}
