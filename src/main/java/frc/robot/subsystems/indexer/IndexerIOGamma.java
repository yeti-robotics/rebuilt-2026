package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class IndexerIOGamma implements IndexerIO {
    public final TalonFX rightMotor;
    public final TalonFX leftMotor;
    public final CANrange topCanRange;
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    public IndexerIOGamma() {
        leftMotor = new TalonFX(IndexerConfigsGamma.LEFT_ROLLER_ID, Constants.rioBus);
        rightMotor = new TalonFX(IndexerConfigsGamma.RIGHT_ROLLER_ID, Constants.rioBus);

        leftMotor.setControl(new Follower(IndexerConfigsGamma.RIGHT_ROLLER_ID, MotorAlignmentValue.Opposed));
        rightMotor.getConfigurator().apply(IndexerConfigsGamma.TalonFXConfigs);

        topCanRange = new CANrange(IndexerConfigsGamma.TOP_CANRANGE_ID, Constants.rioBus);
        topCanRange.getConfigurator().apply(IndexerConfigsGamma.TOP_CANRANGE_CONFIGS);

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(leftMotor);
            PhysicsSim.getInstance().addTalonFX(rightMotor);

        }
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.rollerSpeed = leftMotor.getVelocity().getValueAsDouble();
        inputs.isFull = topCanRange.getIsDetected().getValue();
        inputs.motorTemp = leftMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void spinIndexerRoller(double volts) {
        leftMotor.setControl(new MotionMagicVelocityVoltage(volts));
    }

    @Override
    public void applyPower(double percent) {
        leftMotor.setControl(dutyCycleOut.withOutput(percent));
    }
}
