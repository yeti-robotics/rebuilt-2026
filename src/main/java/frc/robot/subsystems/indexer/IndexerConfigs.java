package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

public class IndexerConfigs {
    static final int INDEXER_MOTOR_ID = 12;
    static final int INDEXER_SENSOR_ID = 25;

    static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(4)
            .withMotionMagicAcceleration(2);

    static final TalonFXConfiguration INDEXER_CONFIGS = new TalonFXConfiguration()
            .withMotionMagic(MOTION_MAGIC_CONFIGS);

    // ask for ratios later - they said they dont have it yet
}
