package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;

public class ShooterConfigsAlpha {
    static final int RIGHT_SHOOTER_ID = 19;
    static final int LEFT_SHOOTER_ID = 0;
    public static final double TEST_SHOOTER_SPEED = 0.8;

    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs()
            .withKP(32)
            .withKI(0)
            .withKD(1)
            .withKS(10)
            .withKV(0.4)
            .withKA(0.5);

    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
            .withMotionMagicAcceleration(128)
            .withMotionMagicCruiseVelocity(4)
            .withMotionMagicJerk(0);

    static final TalonFXConfiguration TOP_MOTOR_CONFIGS = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(2.89).withRotorToSensorRatio(1))
            .withSlot0(SLOT_0_CONFIGS)
            .withMotionMagic(MOTION_MAGIC_CONFIGS)
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    static final TalonFXConfiguration BOTTOM_MOTOR_CONFIGS = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(2.89).withRotorToSensorRatio(1))
            .withSlot0(SLOT_0_CONFIGS)
            .withMotionMagic(MOTION_MAGIC_CONFIGS);
}
