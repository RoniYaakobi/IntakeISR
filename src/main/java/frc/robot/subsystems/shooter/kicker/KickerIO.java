package frc.robot.subsystems.shooter.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {

    @AutoLog
    public static class KickerInputs {
        double kickerSpeedMPS;
    }

    void setDutyCycle(double dutycycle);

    void stop();

    void updateInputs(KickerInputs inputs);
}
