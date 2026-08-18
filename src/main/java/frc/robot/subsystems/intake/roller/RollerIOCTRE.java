package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.hardware.TalonFX;

public class RollerIOCTRE implements RollerIO{

    private final TalonFX talon;

    public RollerIOCTRE(){
        talon = new TalonFX(RollerConstants.ATTRIBUTES.CAN_ID());
        talon.getConfigurator().apply(RollerConstants.getRollerConfig());
    }

    @Override
    public void updateInputs(RollerInputs inputs) {
        inputs.speedMPS = talon.getVelocity().getValueAsDouble();
    }

    @Override
    public void stop() {
        talon.stopMotor();
    }

    @Override
    public void setDutyCycle(double dutycycle) {
        talon.set(dutycycle);
    }

}
