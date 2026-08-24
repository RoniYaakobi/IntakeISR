package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface TwindexerIO {

    @AutoLog
    public static class TwindexerInputs{
        double[] speedsRPS; // The speeds of the spindexers in the twindexer 
    }

    /**
     * Set the dutyCycle for the twindexer
     * @param dutyCycle The dutycycle to apply
     */
    void setDutyCycle(double dutyCycle);

    /**
     * Stop the indexer
     */
    void stop();

    /**
     * Update the indexer inputs object for advantage kit
     * @param inputs The inputs object
     */
    void updateInputs(TwindexerInputs inputs);

}
