package frc.robot.subsystems.shooter.hood;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.math.UnitConversions;
import frc.lib.motor.MotorAttributes;
import frc.robot.Constants;


public class HoodConstants {
    public static final MotorAttributes ATTRIBUTES = 
            new MotorAttributes(19, DCMotor.getNeoVortex(1),
             40, 1, false);

    public static final double MOMENT_OF_INERTIA = 0.015625;

    public static final Rotation2d TOLERANCE_ROTATIONS = Rotation2d.fromDegrees(0.5);

    public static final double MAX_ANGLE = 0.8333;
    public static final double MIN_ANGLE = UnitConversions.degreesToRotations(0.5);

    public static final double STARTING_ANGLE = UnitConversions.degreesToRotations(0);
    public static final double ARM_LENGTH_METERS = 0.2;

    public static final boolean HOOD_SIMULATE_GRAVITY = false;

    public static SparkBaseConfig getHoodConfig(){
        var sparky = new SparkFlexConfig();

        sparky.smartCurrentLimit(60);

        sparky.closedLoop.pid(25, 1, 0);
        sparky.closedLoop.iZone(UnitConversions.degreesToRotations(15));
        sparky.closedLoop.iMaxAccum(5);

        sparky.closedLoop.maxOutput(9 / Constants.MOTOR_IDEAL_VOLTAGE); // There isn't a reason to use more than 9 volts
        sparky.closedLoop.minOutput(9 / Constants.MOTOR_IDEAL_VOLTAGE); // in neither direction

        // Enable voltage compensation (this is a pretty low power consumption subsystem)
        sparky.voltageCompensation(Constants.MOTOR_IDEAL_VOLTAGE);

        // Hood max rotation
        sparky.softLimit.forwardSoftLimit(MAX_ANGLE);
        sparky.softLimit.forwardSoftLimitEnabled(true);

        // Hood min rotation
        sparky.softLimit.reverseSoftLimit(MIN_ANGLE);
        sparky.softLimit.reverseSoftLimitEnabled(true);

        return sparky;
    }

    private static LinearSystem<N2, N1, N2> getPlant(){

        var plant = 
                LinearSystemId.createSingleJointedArmSystem(
                        ATTRIBUTES.MOTOR(),
                        MOMENT_OF_INERTIA,
                        ATTRIBUTES.GEAR_RATIO());

        return plant;
    }

    public static SingleJointedArmSim getHoodSim(){
        return new SingleJointedArmSim(
                getPlant(), 
                ATTRIBUTES.MOTOR(), 
                ATTRIBUTES.GEAR_RATIO(), 
                ARM_LENGTH_METERS,
                UnitConversions.rotationsToRadians(MIN_ANGLE),
                UnitConversions.rotationsToRadians(MAX_ANGLE),
                HOOD_SIMULATE_GRAVITY,
                UnitConversions.rotationsToRadians(STARTING_ANGLE));
    }
}
