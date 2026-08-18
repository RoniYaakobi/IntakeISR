package frc.robot.subsystems.shooter.flywheel;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.motor.MotorAttributes;
import frc.robot.subsystems.shooter.kicker.KickerConstants;

public class FlyWheelConstants {
    public static final MotorAttributes LEAD_ATTRIBUTES = 
            new MotorAttributes(50, DCMotor.getNeoVortex(2),
             1, 0.67 * Math.PI, true);

    public static final double kV = 0.31938;
    public static final double kA = 0.023275;

    public static final int FOLLOWER_CAN_ID = 51;
    public static final boolean MOTORS_OPPOSITE = true;

    public static SparkFlexConfig getLeadFlyWheelConfig(){
        var sparky = new SparkFlexConfig();

        sparky.encoder.positionConversionFactor(LEAD_ATTRIBUTES.UNIT_CONVERSION());
        sparky.smartCurrentLimit(80, 50, 4000);

        sparky.closedLoop.feedForward.kS(0.074823);
        sparky.closedLoop.feedForward.kV(0.31938);

        sparky.closedLoop.pid(0.1, 0.001, 0);
        sparky.closedLoop.iZone(0.3);
        sparky.closedLoop.iMaxAccum(2000);

        sparky.closedLoop.maxMotion.cruiseVelocity(80);
        sparky.closedLoop.maxMotion.maxAcceleration(80);

        sparky.closedLoop.feedForward.kS(0.074823, ClosedLoopSlot.kSlot1);
        sparky.closedLoop.feedForward.kV(0.31938, ClosedLoopSlot.kSlot1);

        sparky.closedLoop.pid(0.6, 0.001, 6, ClosedLoopSlot.kSlot1);
        sparky.closedLoop.iZone(0.6,ClosedLoopSlot.kSlot1);
        sparky.closedLoop.iMaxAccum(2000, ClosedLoopSlot.kSlot1);

        sparky.softLimit.reverseSoftLimit(0);
        sparky.softLimit.reverseSoftLimitEnabled(true);

        return sparky;
    }

    public static SparkFlexConfig getFollowerFlyWheelConfig(){
        var sparky = new SparkFlexConfig();

        sparky.smartCurrentLimit(80, 50, 4000);
        
        sparky.follow(LEAD_ATTRIBUTES.CAN_ID(), MOTORS_OPPOSITE);

        return sparky;
    }

    private static LinearSystem<N1, N1, N1> getPlant(){
        double unitConversion = KickerConstants.ATTRIBUTES.UNIT_CONVERSION();

        Per<VoltageUnit, AngularVelocityUnit> kV = 
            Units.Volts.per(Units.RotationsPerSecond)
                .ofNative(FlyWheelConstants.kV * unitConversion);

        Per<VoltageUnit, AngularAccelerationUnit> kA = 
            Units.Volts.per(Units.RotationsPerSecondPerSecond)
                .ofNative(FlyWheelConstants.kA * unitConversion);
        
        LinearSystem<N1, N1, N1> plant = 
            LinearSystemId.identifyVelocitySystem(
                kV.in(Units.VoltsPerRadianPerSecond),
                kA.in(Units.VoltsPerRadianPerSecondSquared)
        );

        return plant;
    }

    public static FlywheelSim getFlyWheelSim(){
        return new FlywheelSim(getPlant(), FlyWheelConstants.LEAD_ATTRIBUTES.MOTOR());
    }
}
