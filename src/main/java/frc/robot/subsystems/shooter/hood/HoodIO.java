package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface HoodIO {

    @AutoLog
    public static class HoodInputs{
        Rotation2d angle;
    }


    void updateInputs(HoodInputs inputs);

    void setAngle(Rotation2d rotation);

    void stop();
}
