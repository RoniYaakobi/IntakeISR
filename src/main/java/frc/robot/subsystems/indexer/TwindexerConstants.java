package frc.robot.subsystems.indexer;

import com.revrobotics.spark.config.SparkMaxConfig;

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
import frc.lib.statemachine.StateMachine.StateName;

public class TwindexerConstants {
    public static final StateName INDEX_STATE_NAME = new StateName("INDEX");
    public static final StateName STOP_INDEXING_NAME = new StateName("STOP_INDEXING");

    public static final double kA = 0.2;
    public static final double kV = 0.37;

    public static final MotorAttributes LEFT_ATTRIBUTES = 
            new MotorAttributes(40, DCMotor.getNEO(1),
            42.0/15, 1, false);

    public static final MotorAttributes RIGHT_ATTRIBUTES = 
            new MotorAttributes(62, DCMotor.getNEO(1),
            42.0/15, 1, true);

    public static SparkMaxConfig getTwindexerLeftConfig(){
        var sparky = new SparkMaxConfig();

        sparky.inverted(LEFT_ATTRIBUTES.IS_INVERTED());

        sparky.smartCurrentLimit(80,40);

        sparky.closedLoop.pid(0.01, 0.001, 0);
        sparky.closedLoop.iMaxAccum(2000);
        sparky.closedLoop.iZone(3);
        sparky.closedLoop.feedForward.kV(0.37);

        return sparky;
    }

    public static SparkMaxConfig getTwindexerRightConfig(){
        var sparky = getTwindexerLeftConfig();
        sparky.inverted(RIGHT_ATTRIBUTES.IS_INVERTED());
        return sparky;
    }

    /**
     * Get the plant for the indexer simulation
     * @return The plant for the indexer simulation
     */
    private static LinearSystem<N1, N1, N1> getPlant(){
        double unitConversion = TwindexerConstants.LEFT_ATTRIBUTES.UNIT_CONVERSION();

        Per<VoltageUnit, AngularVelocityUnit> kV = 
            Units.Volts.per(Units.RotationsPerSecond)
                .ofNative(TwindexerConstants.kV * unitConversion);

        Per<VoltageUnit, AngularAccelerationUnit> kA = 
            Units.Volts.per(Units.RotationsPerSecondPerSecond)
                .ofNative(TwindexerConstants.kA * unitConversion);
        
        LinearSystem<N1, N1, N1> plant = 
            LinearSystemId.identifyVelocitySystem(
                kV.in(Units.VoltsPerRadianPerSecond),
                kA.in(Units.VoltsPerRadianPerSecondSquared)
        );

        return plant;
    }

    /**
     * Get the twindexer simulation
     * @return The twindexer simulation
     */
    public static FlywheelSim getSpindexerSim(){
        return new FlywheelSim(getPlant(), TwindexerConstants.LEFT_ATTRIBUTES.MOTOR());
    }
}
