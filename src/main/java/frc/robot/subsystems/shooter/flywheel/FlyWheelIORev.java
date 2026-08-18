package frc.robot.subsystems.shooter.flywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class FlyWheelIORev implements FlywheelIO {

    private final SparkFlex leadMotor;
    private final SparkFlex followMotor;

    public FlyWheelIORev(){
        leadMotor = new SparkFlex(FlyWheelConstants.LEAD_ATTRIBUTES.CAN_ID(), MotorType.kBrushless);

        leadMotor.configure(
            FlyWheelConstants.getLeadFlyWheelConfig(), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);

        followMotor = new SparkFlex(FlyWheelConstants.FOLLOWER_CAN_ID, MotorType.kBrushless);

        followMotor.configure(
            FlyWheelConstants.getFollowerFlyWheelConfig(), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(FlyWheelInputs inputs) {
        inputs.speedMPS = leadMotor.getPeriodicStatus2().primaryEncoderVelocity;
    }

    @Override
    public void setSpeedMPS(double speedMPS) {
        leadMotor.getClosedLoopController().setSetpoint(speedMPS, ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void stop() {
        leadMotor.stopMotor();
    }
}
