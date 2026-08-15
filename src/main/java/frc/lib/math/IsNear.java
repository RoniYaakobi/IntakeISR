package frc.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;

public class IsNear {
    public static boolean isNear(double x, double y, double tolerance){
        return Math.abs(x - y) <= tolerance;
    }

    public static boolean isNear(Rotation2d x, Rotation2d y, Rotation2d tolerance){
        return isNear(x.getDegrees(), y.getDegrees(), tolerance.getDegrees());
    }
}
