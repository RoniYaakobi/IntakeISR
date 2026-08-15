package frc.robot.subsystems.shooter.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {

    @AutoLog
    public static class KickerInputs {

    }

    void setDutyCycle(double dutycycle);

    void stop();

    void updateInputs(KickerInputs inputs);
}
