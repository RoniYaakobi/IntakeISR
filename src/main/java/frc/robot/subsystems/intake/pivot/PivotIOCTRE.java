package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;

public class PivotIOCTRE implements PivotIO{

    private final TalonFX talon;

    public PivotIOCTRE(){
        talon = new TalonFX(PivotConstants.ATTRIBUTES.CAN_ID());
        talon.getConfigurator().apply(PivotConstants.getTalonFXConfiguration());
    }

    @Override
    public void setAngle(Rotation2d rotation) {
        talon.setControl(new PositionDutyCycle(rotation.getRadians()));
    }

    @Override
    public void stop() {
        talon.setControl(new StaticBrake());
    }

    @Override
    public void updateInputs(PivotInputs inputs) {
        inputs.position = Rotation2d.fromDegrees(talon.getPosition().getValueAsDouble());
    }
    
}
