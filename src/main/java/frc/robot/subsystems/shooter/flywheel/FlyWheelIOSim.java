package frc.robot.subsystems.shooter.flywheel;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.kicker.KickerConstants;

public class FlyWheelIOSim implements FlywheelIO{

    private final SparkFlex sparky;
    private final SparkFlexSim sparkySim;
    private final FlywheelSim flywheelSim;

    public FlyWheelIOSim(){
        sparky = new SparkFlex(FlyWheelConstants.LEAD_ATTRIBUTES.CAN_ID(), MotorType.kBrushless);
        sparkySim = new SparkFlexSim(sparky, FlyWheelConstants.LEAD_ATTRIBUTES.MOTOR());
        flywheelSim = FlyWheelConstants.getFlyWheelSim();
    }

    @Override
    public void updateInputs(FlyWheelInputs inputs) {
        flywheelSim.setInput(sparkySim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        flywheelSim.update(Constants.LOOP_PERIOD_SECONDS);

        sparkySim.iterate(
            flywheelSim.getAngularVelocityRPM() * KickerConstants.ATTRIBUTES.GEAR_RATIO(),
            RoboRioSim.getVInVoltage(),
            Constants.LOOP_PERIOD_SECONDS);
        
        inputs.speedMPS = sparkySim.getVelocity();
    }

    @Override
    public void setSpeedMPS(double speedMPS) {
        sparky.getClosedLoopController().setSetpoint(speedMPS, ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void stop() {
        sparkySim.disable();
    }

}
