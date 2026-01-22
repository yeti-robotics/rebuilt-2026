package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConfigs.*;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class IndexerIOAlpha implements IndexerIO {
    public final TalonFX indexerMotor;
    public final CANrange indexerSensor;

    public final MotionMagicVelocityVoltage velocityVoltageRequest = new MotionMagicVelocityVoltage(0);

    public IndexerIOAlpha() {
        indexerMotor = new TalonFX(INDEXER_MOTOR_ID, Constants.rioBus);
        indexerSensor = new CANrange(INDEXER_SENSOR_ID, Constants.rioBus);

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(indexerMotor);
        }
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerVoltage = indexerMotor.getMotorVoltage().getValueAsDouble();
        inputs.isDetected = indexerSensor.getIsDetected().getValue();
    }

    @Override
    public void spinIndexer(double volts) {
        indexerMotor.setControl(velocityVoltageRequest.withVelocity(volts));
    }

    @Override
    public boolean getIsDetected() {
        return indexerSensor.getIsDetected().getValue();
    }
}
