package frc.robot.subsystems.intakeISR;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeISRIORev implements IntakeISRIO{

    private final SparkMax pivotLead;
    private final SparkMax pivotFollower;

    private final SparkFlex rollerLead;
    private final SparkFlex rollerFollower;

    public IntakeISRIORev(){
        pivotLead = new SparkMax(6, MotorType.kBrushless);
        pivotFollower = new SparkMax(7, MotorType.kBrushless);

        SparkMaxConfig config = (SparkMaxConfig)(new SparkMaxConfig().apply(
            new ClosedLoopConfig().pid(1, 0, 0)
        ));

        pivotLead.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        pivotFollower.configure(new SparkMaxConfig().follow(6),
         ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        rollerLead = new SparkFlex(16, MotorType.kBrushless);
        rollerFollower = new SparkFlex(17, MotorType.kBrushless);

        rollerLead.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        rollerFollower.configure(new SparkMaxConfig().follow(16),
         ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateInputs(IntakeISRInputs inputs) {
        inputs.positionRotation = Rotation2d.fromRotations(pivotLead.getEncoder().getPosition());
    }

    @Override
    public void goToRotation(Rotation2d rotation) {
        pivotLead.getClosedLoopController().setSetpoint(rotation.getRotations(), ControlType.kPosition);
    }

    @Override
    public void setRollerSpeed(double speedMPS) {
        rollerLead.getClosedLoopController().setSetpoint(speedMPS, ControlType.kVelocity);
    }

    @Override
    public void stopRoller() {
        setRollerSpeed(0);
    }
}
