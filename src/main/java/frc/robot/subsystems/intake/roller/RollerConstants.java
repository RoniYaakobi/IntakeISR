package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

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

public class RollerConstants {
    public static final double ACTIVE_DUTYCYCLE = 1;

    public static final MotorAttributes ATTRIBUTES = 
        new MotorAttributes(
            22, DCMotor.getKrakenX60(1),
            37/11.0, 1, true);

    public static final double kA = 0.023275;
    public static final double kV = 0.31938;

    public static TalonFXConfiguration getRollerConfig(){
        var config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 90;
        config.CurrentLimits.StatorCurrentLimitEnable = false;

        return config;
    }

    private static LinearSystem<N1, N1, N1> getPlant(){
        double unitConversion = KickerConstants.ATTRIBUTES.UNIT_CONVERSION();

        Per<VoltageUnit, AngularVelocityUnit> kV = 
            Units.Volts.per(Units.RotationsPerSecond)
                .ofNative(RollerConstants.kV * unitConversion);

        Per<VoltageUnit, AngularAccelerationUnit> kA = 
            Units.Volts.per(Units.RotationsPerSecondPerSecond)
                .ofNative(RollerConstants.kA * unitConversion);
        
        LinearSystem<N1, N1, N1> plant = 
            LinearSystemId.identifyVelocitySystem(
                kV.in(Units.VoltsPerRadianPerSecond),
                kA.in(Units.VoltsPerRadianPerSecondSquared)
        );

        return plant;
    }

    public static FlywheelSim getFlyWheelSim(){
        return new FlywheelSim(getPlant(), RollerConstants.ATTRIBUTES.MOTOR());
    }
    
}
