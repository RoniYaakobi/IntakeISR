package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.math.UnitConversions;
import frc.robot.Constants;
import frc.robot.subsystems.intake.roller.RollerConstants;

public class PivotIOSim implements PivotIO {

    private final TalonFX talon;
    private final SingleJointedArmSim pivotSim;
    public PivotIOSim(){
        talon = new TalonFX(PivotConstants.ATTRIBUTES.CAN_ID());
        talon.getConfigurator().apply(PivotConstants.getTalonFXConfiguration());

        pivotSim = PivotConstants.getPivotSim();
    }

    @Override
    public void setAngle(Rotation2d rotation) {
        talon.setControl(new PositionDutyCycle(rotation.getRadians()));
    }

    @Override
    public void stop() {
        talon.stopMotor();
    }

    @Override
    public void updateInputs(PivotInputs inputs) {
        var talonFXSim = talon.getSimState();

        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        pivotSim.setInputVoltage(motorVoltage.in(Units.Volts));
        pivotSim.update(Constants.LOOP_PERIOD_SECONDS);

        talonFXSim.setRawRotorPosition(UnitConversions.radiansToRotations(pivotSim.getAngleRads()));

        talonFXSim.setRotorVelocity(UnitConversions.radiansToRotations(pivotSim.getVelocityRadPerSec()) * RollerConstants.ATTRIBUTES.GEAR_RATIO());
        inputs.position = Rotation2d.fromRotations(talon.getPosition().getValueAsDouble());
    }
    
}
