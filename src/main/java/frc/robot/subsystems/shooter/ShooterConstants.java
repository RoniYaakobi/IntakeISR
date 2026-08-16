package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.statemachine.StateMachine.StateName;

public class ShooterConstants {

    public record ShootParams(double flywheelspeedMPS, double dutycycleKicker, Rotation2d hoodAngle) {}

    public static final Transform3d ROBOT_TO_SHOOTER = new Transform3d(0, 0, 0, Rotation3d.kZero);

    public static final boolean SHOOT_WITH_MOVEMENT = true;
    public static final double ANGULAR_SPEED_DEADBAND_DEGREES = 5;
    public static final double LINEAR_SPEED_DEADBAND_MPS = 0.3;

    public static final Rotation2d HOOD_TOLERANCE = Rotation2d.fromDegrees(0.1);
    public static final double FLYWHEEL_MPS_TOLERANCE = 0.5;

    public static final StateName SPIN_UP_AND_SHOOT_STATE_NAME = new StateName("SPIN_UP_AND_SHOOT");
    public static final StateName STOP_SHOOTING_STATE_NAME = new StateName("STOP_SHOOTING");
    public static final StateName SPIN_UP = new StateName("SPIN_UP");
    public static final StateName SHOOT = new StateName("SHOOT");

    public static final StateName ZERO_HOOD = new StateName("ZERO_HOOD");
    public static final StateName CLOSE_HOOD = new StateName("CLOSE_HOOD");


}
