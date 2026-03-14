package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.UpdateModeValue;

public class IndexerConfigsBeta {
    static final int INDEXER_MOTOR_ID = 50;
    static final int INDEXER_CANRANGE_ID = 51;
    public static final double TEST_INDEXER_SPEED = 0.9;

    static final MotionMagicConfigs MOTION_MAGIC_CONFIGS =
            new MotionMagicConfigs().withMotionMagicCruiseVelocity(1).withMotionMagicAcceleration(2);

    static final TalonFXConfiguration INDEXER_MOTOR_CONFIGS = new TalonFXConfiguration()
            .withMotionMagic(MOTION_MAGIC_CONFIGS)
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(100)
                    .withSupplyCurrentLimit(100)
                    .withStatorCurrentLimitEnable(false)
                    .withSupplyCurrentLimitEnable(false)
                    .withSupplyCurrentLowerLimit(40)
                    .withSupplyCurrentLowerTime(1));

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
