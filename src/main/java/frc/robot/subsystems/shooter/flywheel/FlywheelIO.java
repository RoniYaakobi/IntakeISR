package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;


public interface FlywheelIO {

    @AutoLog
    public static class FlyWheelInputs{

    }


    void updateInputs(FlyWheelInputs inputs);

    void setSpeedMPS(double speedMPS);

    void stop();
}
