package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class TwindexerIORev implements TwindexerIO {

    private final SparkMax[] sparkies;

    public TwindexerIORev(){
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
            speedsRPS[i] = sparkies[i].getPeriodicStatus2().primaryEncoderVelocity;
        }

        inputs.speedsRPS = speedsRPS;
    }
}
