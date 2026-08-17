package frc.robot.subsystems.shooter.kicker;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;

public class KickerIOSim implements KickerIO{

    private final SparkFlex sparky;
    private final SparkFlexSim sparkySim;
    private final FlywheelSim kickerSim;

    public KickerIOSim(){
        sparky = new SparkFlex(KickerConstants.CAN_ID, MotorType.kBrushless);
        sparkySim = new SparkFlexSim(sparky, KickerConstants.MOTOR);
        sparky.configure(KickerConstants.getKickerConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        kickerSim = KickerConstants.getKickerSim();
    }


    @Override
    public void setDutyCycle(double dutycycle) {
        sparkySim.enable(); //TODO check this in more depth
        sparkySim.setAppliedOutput(dutycycle);
    }

    @Override
    public void stop() {
        sparkySim.disable();
    }

    @Override
    public void updateInputs(KickerInputs inputs) {
        kickerSim.setInput(sparkySim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        kickerSim.update(Constants.LOOP_PERIOD_SECONDS);

        sparkySim.iterate(
            kickerSim.getAngularVelocityRPM() * KickerConstants.GEAR_RATIO,
            RoboRioSim.getVInVoltage(),
            Constants.LOOP_PERIOD_SECONDS);
        
        inputs.kickerSpeedMPS = sparkySim.getVelocity();
    }

}
