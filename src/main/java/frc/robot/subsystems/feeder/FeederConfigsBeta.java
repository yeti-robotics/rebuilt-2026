package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.UpdateModeValue;

public class FeederConfigsBeta {
    static final int FEEDER_MOTOR_ID = 50;
    static final int FEEDER_CANRANGE_ID = 51;
    public static final double TEST_FEEDER_SPEED = 0.9;

    static final MotionMagicConfigs MOTION_MAGIC_CONFIGS =
            new MotionMagicConfigs().withMotionMagicCruiseVelocity(1).withMotionMagicAcceleration(2);

    static final TalonFXConfiguration FEEDER_MOTOR_CONFIGS =
            new TalonFXConfiguration().withMotionMagic(MOTION_MAGIC_CONFIGS);

    static final CANrangeConfiguration CANRANGE_CONFIGS = new CANrangeConfiguration()
            .withToFParams(new ToFParamsConfigs().withUpdateMode(UpdateModeValue.ShortRange100Hz))
            .withFovParams(new FovParamsConfigs()
                    .withFOVCenterX(0)
                    .withFOVCenterY(0)
                    .withFOVRangeX(27)
                    .withFOVRangeY(27))
            .withProximityParams(new ProximityParamsConfigs()
                    .withProximityThreshold(0.15)
                    .withProximityHysteresis(0.1)
                    .withMinSignalStrengthForValidMeasurement(2500));
}
