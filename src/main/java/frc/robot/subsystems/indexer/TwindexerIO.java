package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface TwindexerIO {

    @AutoLog
    public static class TwindexerInputs{
        double[] speedsRPS; // The speeds of the spindexers in the twindexer 
    }

    /**
     * Set the dutyCycle for the Twindexers
     * @param dutyCycle
     */
    void setDutyCycle(double dutyCycle);

    void stop();

    void updateInputs(TwindexerInputs inputs);

}
