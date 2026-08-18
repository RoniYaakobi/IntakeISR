package frc.robot.subsystems.shooter.hood;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;

public class HoodIORev implements HoodIO {

    private final SparkFlex sparky;

    public HoodIORev(){
        sparky = new SparkFlex(HoodConstants.ATTRIBUTES.CAN_ID(), MotorType.kBrushless);
        sparky.configure(HoodConstants.getHoodConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(HoodInputs inputs) {
        inputs.angle = Rotation2d.fromRotations(sparky.getPeriodicStatus2().primaryEncoderPosition);
    }

    @Override
    public void setAngle(Rotation2d rotation) {
        sparky.getClosedLoopController().setSetpoint(rotation.getRotations(), ControlType.kPosition);
    }

    @Override
    public void stop() {
        sparky.stopMotor();
    }

}
