package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;


public interface FlywheelIO {

    @AutoLog
    public static class FlyWheelInputs{
        double speedMPS;
    }

    /**
     * Update the inputs object for the flywheel 
     * @param inputs The flywheel inputs object
     */
    void updateInputs(FlyWheelInputs inputs);

    /**
     * Set the flywheel speed in MPS
     * @param speedMPS The speed at which you want the flywheel to spin
     */
    void setSpeedMPS(double speedMPS);

    /**
     * Stop the flywheel from spinning
     */
    void stop();
}
