package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.UpdateModeValue;

public class IndexerConfigs {
    static final int INDEXER_MOTOR_ID = 32;
    static final int INDEXER_CANRANGE_ID = 25;
    public static final int TEST_INDEXER_SPEED = 0.2;

    static final MotionMagicConfigs MOTION_MAGIC_CONFIGS =
            new MotionMagicConfigs().withMotionMagicCruiseVelocity(4).withMotionMagicAcceleration(2);

    static final TalonFXConfiguration INDEXER_MOTOR_CONFIGS =
            new TalonFXConfiguration().withMotionMagic(MOTION_MAGIC_CONFIGS);

    // ask for ratios later - they said they dont have it yet

    // placeholder values
    static final CANrangeConfiguration CANRANGE_CONFIGS = new CANrangeConfiguration()
            .withToFParams(new ToFParamsConfigs().withUpdateMode(UpdateModeValue.ShortRange100Hz))
            .withFovParams(new FovParamsConfigs()
                    .withFOVCenterX(11.8)
                    .withFOVCenterY(11.8)
                    .withFOVRangeX(6.75)
                    .withFOVRangeY(6.75))
            .withProximityParams(
                    new ProximityParamsConfigs().withProximityThreshold(0.17).withProximityHysteresis(0.02));
}
