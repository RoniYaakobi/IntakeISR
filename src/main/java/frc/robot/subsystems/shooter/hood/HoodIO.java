package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface HoodIO {

    @AutoLog
    public static class HoodInputs{
        Rotation2d angle;
    }


    /**
     * Update the inputs object for advantage kit replay
     * @param inputs The inputs object to update
     */
    void updateInputs(HoodInputs inputs);

    /**
     * Set the hood angle
     * @param rotation The angle that the hood needs to go to.
     */
    void setAngle(Rotation2d rotation);

    /**
     * Stop the hood motor.
     */
    void stop();
}
