package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface TwindexerIO {

    @AutoLog
    public static class TwindexerInputs{
        
    }

    void setDutyCycle(double dutyCycle);

    void stop();

    void updateInputs(TwindexerInputs inputs);

}
