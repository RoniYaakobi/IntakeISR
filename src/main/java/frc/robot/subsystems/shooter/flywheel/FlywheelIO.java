package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;


public interface FlywheelIO {

    @AutoLog
    public static class FlyWheelInputs{
        double speedMPS;
    }


    void updateInputs(FlyWheelInputs inputs);

    void setSpeedMPS(double speedMPS);

    void stop();
}
