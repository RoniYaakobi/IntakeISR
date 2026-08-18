package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.math.UnitConversions;
import frc.robot.Constants;

public class RollerIOSim implements RollerIO {

    private final TalonFX talon;
    private final FlywheelSim flywheelSim;

    public RollerIOSim(){
        talon = new TalonFX(RollerConstants.ATTRIBUTES.CAN_ID());
        talon.getConfigurator().apply(RollerConstants.getRollerConfig());

        talon.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

        flywheelSim = RollerConstants.getFlyWheelSim();
    }

    @Override
    public void updateInputs(RollerInputs inputs) {
        var talonFXSim = talon.getSimState();

        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        flywheelSim.setInputVoltage(motorVoltage.in(Units.Volts));
        flywheelSim.update(Constants.LOOP_PERIOD_SECONDS);

        talonFXSim.addRotorPosition(
            UnitConversions.RPMtoRotationsPerCycle(
                flywheelSim.getAngularVelocityRPM(), 
                Constants.LOOP_PERIOD_SECONDS) * RollerConstants.ATTRIBUTES.GEAR_RATIO()
        );

        talonFXSim.setRotorVelocity(flywheelSim.getAngularVelocity().times(RollerConstants.ATTRIBUTES.GEAR_RATIO()));
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
