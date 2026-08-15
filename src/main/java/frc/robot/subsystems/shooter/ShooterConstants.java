package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class ShooterConstants {

    public record ShootParams(double flywheelspeedMPS, double dutycycleKicker, Rotation2d hoodAngle) {}

    public static final Transform3d ROBOT_TO_SHOOTER = new Transform3d(0, 0, 0, Rotation3d.kZero);

    public static final boolean SHOOT_WITH_MOVEMENT = true;
    public static final double ANGULAR_SPEED_DEADBAND_DEGREES = 5;
    public static final double LINEAR_SPEED_DEADBAND_MPS = 0.3;

    public static final Rotation2d HOOD_TOLERANCE = Rotation2d.fromDegrees(0.1);
}
