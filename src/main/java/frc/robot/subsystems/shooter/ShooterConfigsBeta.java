package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;

public class ShooterConfigsBeta {
    static final int RIGHT_SHOOTER_ID = 54;
    static final int LEFT_SHOOTER_ID = 55;
    public static final double TEST_SHOOTER_SPEED = 0.8;

    public static final Slot0Configs SLOT_0_CONFIGS =
            new Slot0Configs().withKP(0).withKI(0).withKD(0).withKA(0).withKV(0).withKS(0);

    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
            .withMotionMagicAcceleration(2)
            .withMotionMagicCruiseVelocity(1)
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
