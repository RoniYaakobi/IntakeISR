package frc.robot.subsystems.shooter.hood;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.math.UnitConversions;
import frc.robot.Constants;

public class HoodIOSim implements HoodIO {
    private final SparkFlex sparky;
    private final SparkFlexSim sparkySim;
    private final SingleJointedArmSim hoodSim;

    public HoodIOSim(){
        sparky = new SparkFlex(HoodConstants.ATTRIBUTES.CAN_ID(), MotorType.kBrushless);
        sparkySim = new SparkFlexSim(sparky, HoodConstants.ATTRIBUTES.MOTOR());
        sparky.configure(HoodConstants.getHoodConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        hoodSim = HoodConstants.getHoodSim();
    }

    @Override
    public void updateInputs(HoodInputs inputs) {
        hoodSim.setInput(sparkySim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        hoodSim.update(Constants.LOOP_PERIOD_SECONDS);

        sparkySim.iterate(
            UnitConversions.radiansPerSecondToRotationsPerMinute(
                hoodSim.getVelocityRadPerSec()) * HoodConstants.ATTRIBUTES.GEAR_RATIO(),
            RoboRioSim.getVInVoltage(),
            Constants.LOOP_PERIOD_SECONDS);

        inputs.angle = Rotation2d.fromRadians(hoodSim.getAngleRads());
    }

    @Override
    public void setAngle(Rotation2d rotation) {
        sparky.getClosedLoopController().setSetpoint(rotation.getRotations(), ControlType.kPosition);
    }

    @Override
    public void stop() {
        sparkySim.disable();
    }
}
