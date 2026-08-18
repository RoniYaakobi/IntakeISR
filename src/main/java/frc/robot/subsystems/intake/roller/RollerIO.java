package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLog;


public interface RollerIO {
    
    @AutoLog
    public static class RollerInputs{
        double speedMPS;
    }

    void updateInputs(RollerInputs inputs);

    void stop();

    void setDutyCycle(double dutycycle);
}
