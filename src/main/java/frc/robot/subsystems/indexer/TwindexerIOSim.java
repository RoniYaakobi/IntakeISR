package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.kicker.KickerConstants;

import com.revrobotics.spark.SparkMax;

public class TwindexerIOSim implements TwindexerIO{
    private final SparkMax[] sparkies;
    private final SparkMaxSim[] sparkySims;

    private final FlywheelSim[] twindexerSim;

    public TwindexerIOSim(){
        sparkies = new SparkMax[]{
            new SparkMax(TwindexerConstants.LEFT_ATTRIBUTES.CAN_ID(), MotorType.kBrushless),
            new SparkMax(TwindexerConstants.RIGHT_ATTRIBUTES.CAN_ID(), MotorType.kBrushless)
        };

        sparkies[0].configure(
            TwindexerConstants.getTwindexerLeftConfig(),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        sparkies[1].configure(
            TwindexerConstants.getTwindexerRightConfig(),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        sparkySims = new SparkMaxSim[2];

        for (int i = 0; i < sparkies.length; i++){
            sparkySims[i] = new SparkMaxSim(sparkies[i], TwindexerConstants.LEFT_ATTRIBUTES.MOTOR());
        }

        twindexerSim = new FlywheelSim[]{TwindexerConstants.getSpindexerSim(), TwindexerConstants.getSpindexerSim()};
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        for (SparkMax sparky : sparkies){
            sparky.set(dutyCycle);
        }
    }

    @Override
    public void stop() {
        for (SparkMax sparky : sparkies){
            sparky.stopMotor();
        }
    }

    @Override
    public void updateInputs(TwindexerInputs inputs) {
        double[] speedsRPS = new double[2];
        for (int i = 0; i < sparkies.length; i++){
            twindexerSim[i].setInput(sparkySims[0].getAppliedOutput() * RoboRioSim.getVInVoltage());
            twindexerSim[i].update(Constants.LOOP_PERIOD_SECONDS);

            sparkySims[i].iterate(
                    twindexerSim[i].getAngularVelocityRPM() * KickerConstants.ATTRIBUTES.GEAR_RATIO(),
                    RoboRioSim.getVInVoltage(),
                    Constants.LOOP_PERIOD_SECONDS);

            speedsRPS[i] = sparkies[i].getPeriodicStatus2().primaryEncoderVelocity;
        }

        inputs.speedsRPS = speedsRPS;
    }
}
