package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.motor.MotorAttributes;

public class PivotConstants {
    public static final Rotation2d OPEN_PIVOT_POSITION = Rotation2d.kZero;
    public static final Rotation2d CLOSE_PIVOT_POSITION = Rotation2d.fromRotations(150);
    public static final Rotation2d POSITION_TOLERANCE = Rotation2d.fromDegrees(3);


    public static final MotorAttributes ATTRIBUTES = 
        new MotorAttributes(42, DCMotor.getKrakenX60Foc(1),
        5.4, 360, false);

    public static final double MOMENT_OF_INERTIA = 1.0;
    public static final double INTAKE_LENGTH_METERS = 0.6;
    public static final boolean SIMULATE_ARM_GRAVITY = true;

    public static TalonFXConfiguration getTalonFXConfiguration(){
        var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Feedback.SensorToMechanismRatio = ATTRIBUTES.GEAR_RATIO();

        config.Slot0.kP = 30;
        config.Slot0.kI = 0.5;

        config.MotionMagic.MotionMagicCruiseVelocity = 10;
        config.MotionMagic.MotionMagicAcceleration = 10;
    
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        return config;
    }

    private static LinearSystem<N2, N1, N2> getPlant(){

        var plant = 
                LinearSystemId.createSingleJointedArmSystem(
                        ATTRIBUTES.MOTOR(),
                        MOMENT_OF_INERTIA,
                        ATTRIBUTES.GEAR_RATIO());

        return plant;
    }

    public static SingleJointedArmSim getPivotSim(){
        return new SingleJointedArmSim(
                getPlant(), 
                ATTRIBUTES.MOTOR(), 
                ATTRIBUTES.GEAR_RATIO(), 
                INTAKE_LENGTH_METERS,
                CLOSE_PIVOT_POSITION.getRadians(),
                OPEN_PIVOT_POSITION.getRadians(),
                SIMULATE_ARM_GRAVITY,
                CLOSE_PIVOT_POSITION.getRadians());
    }


}
