package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public class PivotConstants {
    public static final Rotation2d OPEN_PIVOT_POSITION = Rotation2d.kZero;
    public static final Rotation2d CLOSE_PIVOT_POSITION = Rotation2d.fromRotations(150);
    public static final Rotation2d POSITION_TOLERANCE = Rotation2d.fromDegrees(3);
}
