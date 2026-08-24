package frc.robot.subsystems.shooter.kicker;

import com.revrobotics.spark.config.SparkBaseConfig;
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

public class KickerConstants {
    // Attributes of the motor
    public static final MotorAttributes ATTRIBUTES = 
            new MotorAttributes(30, DCMotor.getNeoVortex(1),
             3, 0.67 * Math.PI, false);

    // Simulation constants
    public static final double kV = 0.31938;
    public static final double kA = 0.023275;

    // Kicker tolerance to setpoint in mps
    public static final double TOLERANCE_MPS = 0.5;

    public static SparkBaseConfig getKickerConfig(){
        var sparky = new SparkFlexConfig();

        sparky.smartCurrentLimit(80, 40);

        sparky.encoder.positionConversionFactor(ATTRIBUTES.UNIT_CONVERSION());

        sparky.closedLoop.pid(0.1, 0.05, 0);
        sparky.closedLoop.iZone(2);
        sparky.closedLoop.iMaxAccum(60);

        sparky.closedLoop.feedForward.kV(0.382 / ATTRIBUTES.UNIT_CONVERSION());

        return sparky;
    }

    /**
     * Get the plant for the kicker simulation
     * @return The plant for the kicker simulation.
     */
    private static LinearSystem<N1, N1, N1> getPlant(){
        double unitConversion = KickerConstants.ATTRIBUTES.UNIT_CONVERSION();

        Per<VoltageUnit, AngularVelocityUnit> kV = 
            Units.Volts.per(Units.RotationsPerSecond)
                .ofNative(KickerConstants.kV * unitConversion);

        Per<VoltageUnit, AngularAccelerationUnit> kA = 
            Units.Volts.per(Units.RotationsPerSecondPerSecond)
                .ofNative(KickerConstants.kA * unitConversion);
        
        // Process in state space notation
        LinearSystem<N1, N1, N1> plant = 
            LinearSystemId.identifyVelocitySystem(
                kV.in(Units.VoltsPerRadianPerSecond),
                kA.in(Units.VoltsPerRadianPerSecondSquared)
        );

        return plant;
    }

    /**
     * Get a flywheel simulation
     * @return The simulation.
     */
    public static FlywheelSim getKickerSim(){
        return new FlywheelSim(getPlant(), KickerConstants.ATTRIBUTES.MOTOR());
    }
}
