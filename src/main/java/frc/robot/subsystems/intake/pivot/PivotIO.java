package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface PivotIO {

    @AutoLog
    public static class PivotInputs{
        Rotation2d position;
    }

    void setAngle(Rotation2d rotation);

    void stop();

    void updateInputs(PivotInputs inputs);
}
