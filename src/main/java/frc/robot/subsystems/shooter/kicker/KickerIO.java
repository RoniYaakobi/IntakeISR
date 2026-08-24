package frc.robot.subsystems.shooter.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {

    @AutoLog
    public static class KickerInputs {
        double kickerSpeedMPS;
    }

    /**
     * Set the dutycycle of the kicker motor
     * @param dutycycle The dutycycle to apply to the motor.
     */
    void setDutyCycle(double dutycycle);

    /**
     * Stop the kicker motor.
     */
    void stop();

    /**
     * Update the inputs object of the Kicker
     * @param inputs The inputs object that needs to be updated.
     */
    void updateInputs(KickerInputs inputs);
}
