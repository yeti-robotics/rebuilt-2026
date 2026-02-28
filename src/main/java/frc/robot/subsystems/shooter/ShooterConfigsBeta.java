package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.robot.subsystems.hood.HoodPositions;
import frc.robot.util.ShooterStateData;

public class ShooterConfigsBeta {
    static final int RIGHT_SHOOTER_ID = 54;
    static final int LEFT_SHOOTER_ID = 55;
    public static final double TEST_SHOOTER_SPEED = 0.6;

    static final double ROTOR_TO_SENSOR = 1;
    static final double SENSOR_TO_MECHANISM = 1;

    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs()
            .withKP(16)
            .withKI(0)
            .withKD(0)
            .withKS(8.5)
            .withKV(0.2)
            .withKA(1.5);

    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
            .withMotionMagicAcceleration(256)
            .withMotionMagicCruiseVelocity(1)
            .withMotionMagicJerk(0);

    static final TalonFXConfiguration TOP_MOTOR_CONFIGS = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(SENSOR_TO_MECHANISM)
                    .withRotorToSensorRatio(ROTOR_TO_SENSOR))
            .withSlot0(SLOT_0_CONFIGS)
            .withMotionMagic(MOTION_MAGIC_CONFIGS)
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    static final TalonFXConfiguration BOTTOM_MOTOR_CONFIGS = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(SENSOR_TO_MECHANISM)
                    .withRotorToSensorRatio(ROTOR_TO_SENSOR))
            .withSlot0(SLOT_0_CONFIGS)
            .withMotionMagic(MOTION_MAGIC_CONFIGS);

    public static final InterpolatingTreeMap<Double, ShooterStateData> SHOOTER_MAP =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterStateData.interpolator);

    static {
        SHOOTER_MAP.put(
                2.08, new ShooterStateData(HoodPositions.STOW.getPosition(), 25, 0.0)); // hood, rps, flight time
        SHOOTER_MAP.put(3.38, new ShooterStateData(HoodPositions.STOW.getPosition(), 33, 0.0));
        SHOOTER_MAP.put(4.31, new ShooterStateData(HoodPositions.STOW.getPosition(), 35, 0.0));
        SHOOTER_MAP.put(2.58, new ShooterStateData(HoodPositions.STOW.getPosition(), 27, 0.0));
        SHOOTER_MAP.put(3.74, new ShooterStateData(HoodPositions.STOW.getPosition(), 34, 0.0));
        SHOOTER_MAP.put(4.46, new ShooterStateData(HoodPositions.STOW.getPosition(), 42, 0.0));
    }
}
