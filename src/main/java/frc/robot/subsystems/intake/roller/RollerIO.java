package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLog;


public interface RollerIO {
    
    @AutoLog
    public static class RollerInputs{
        double speedMPS;
    }

    /**
     * Update the inputs object for advantage kit replay
     * @param inputs The inputs object
     */
    void updateInputs(RollerInputs inputs);

    /**
     * Stop the intake roller
     */
    void stop();

    /**
     * Set a dutycycle for the intake roller
     * @param dutycycle The dutycycle to apply
     */
    void setDutyCycle(double dutycycle);
}
