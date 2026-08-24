package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.statemachine.StateMachine.StateName;

public class ShooterConstants {

    // Used to store the shooting parameters
    public record ShootParams(double flywheelspeedMPS, double dutycycleKicker, Rotation2d hoodAngle) {}

    // Transformation between the robot and the shooter
    public static final Transform3d ROBOT_TO_SHOOTER = new Transform3d(0, 0, 0, Rotation3d.kZero);

    // The tolerances of the subsystem to be considered at the goal
    public static final Rotation2d HOOD_TOLERANCE = Rotation2d.fromDegrees(0.1);
    public static final double FLYWHEEL_MPS_TOLERANCE = 0.5;

    // The superstructure states
    public static final StateName SPIN_UP_AND_SHOOT_STATE_NAME = new StateName("SPIN_UP_AND_SHOOT");
    public static final StateName STOP_SHOOTING_STATE_NAME = new StateName("STOP_SHOOTING");

    // The shooting and spinup states
    public static final StateName SPIN_UP = new StateName("SPIN_UP");
    public static final StateName SHOOT = new StateName("SHOOT");
}
