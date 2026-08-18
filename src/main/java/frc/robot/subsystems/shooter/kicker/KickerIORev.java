package frc.robot.subsystems.shooter.kicker;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class KickerIORev implements KickerIO{

    private final SparkFlex sparky;

    public KickerIORev(){
        sparky = new SparkFlex(KickerConstants.ATTRIBUTES.CAN_ID(), MotorType.kBrushless);
        sparky.configure(KickerConstants.getKickerConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setDutyCycle(double dutycycle) {
        sparky.set(dutycycle);
    }

    @Override
    public void stop() {
        sparky.stopMotor();
    }

    @Override
    public void updateInputs(KickerInputs inputs) {
        inputs.kickerSpeedMPS = sparky.getPeriodicStatus2().primaryEncoderVelocity;
    }

}
