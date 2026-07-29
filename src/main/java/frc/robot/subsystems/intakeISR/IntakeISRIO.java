package frc.robot.subsystems.intakeISR;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeISRIO {

    @AutoLog
    public class IntakeISRInputs{
        public Rotation2d positionRotation;
    }

    void updateInputs(IntakeISRInputs inputs);

    // Set the pivot to an angle
    void goToRotation(Rotation2d positionMeters);

    // Set the roller to a speed
    void setRollerSpeed(double speedMPS);

    // Stop the roller
    void stopRoller();
}
