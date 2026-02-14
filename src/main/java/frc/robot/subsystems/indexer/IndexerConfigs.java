package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.UpdateModeValue;

public class IndexerConfigs {
    static final int INDEXER_MOTOR_ID = 32;
    static final int INDEXER_CANRANGE_ID = 25;
    public static final double TEST_INDEXER_SPEED = 0.2;

    static final MotionMagicConfigs MOTION_MAGIC_CONFIGS =
            new MotionMagicConfigs().withMotionMagicCruiseVelocity(4).withMotionMagicAcceleration(2);

    static final TalonFXConfiguration INDEXER_MOTOR_CONFIGS =
            new TalonFXConfiguration().withMotionMagic(MOTION_MAGIC_CONFIGS);

    // ask for ratios later - they said they dont have it yet

    // placeholder values
    static final CANrangeConfiguration CANRANGE_CONFIGS = new CANrangeConfiguration()
            .withToFParams(new ToFParamsConfigs().withUpdateMode(UpdateModeValue.ShortRange100Hz))
            .withFovParams(new FovParamsConfigs()
                    .withFOVCenterX(0)
                    .withFOVCenterY(0)
                    .withFOVRangeX(27)
                    .withFOVRangeY(27))
            .withProximityParams(new ProximityParamsConfigs()
                    .withProximityThreshold(0.126)
                    .withProximityHysteresis(0.1)
                    .withMinSignalStrengthForValidMeasurement(2500));
}
