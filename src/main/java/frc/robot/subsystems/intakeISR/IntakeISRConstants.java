package frc.robot.subsystems.intakeISR;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeISRConstants {
    // Open/Close angle and tolerances
    public static final Rotation2d OPEN_ROTATION = Rotation2d.fromRotations(0.67);
    public static final Rotation2d OPEN_TOLERANCE = Rotation2d.fromDegrees(6.7);
    public static final Rotation2d CLOSED_ROTATION = Rotation2d.fromRotations(0.0);

    // The speed of the roller in MPS
    public static final double ROLLER_SPEED_RPS = 67;
}
