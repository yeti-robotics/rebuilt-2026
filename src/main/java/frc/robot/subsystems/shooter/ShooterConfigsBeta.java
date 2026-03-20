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
        SHOOTER_MAP.put(1.74, new ShooterStateData(HoodPositions.STOW.getPosition(), 24, 0.0));
        SHOOTER_MAP.put(2.13, new ShooterStateData(HoodPositions.STOW.getPosition(), 26, 0.0));
        SHOOTER_MAP.put(2.43, new ShooterStateData(HoodPositions.STOW.getPosition(), 29, 0.0));
        SHOOTER_MAP.put(2.78, new ShooterStateData(HoodPositions.STOW.getPosition(), 30, 0.0));
        SHOOTER_MAP.put(3.15, new ShooterStateData(HoodPositions.STOW.getPosition(), 32, 0.0));
        SHOOTER_MAP.put(3.78, new ShooterStateData(HoodPositions.STOW.getPosition(), 34, 0.0));
        SHOOTER_MAP.put(4.36, new ShooterStateData(HoodPositions.STOW.getPosition(), 38.5, 0.0));
    }

    public static final InterpolatingTreeMap<Double, ShooterStateData> SHUTTLE_MAP =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterStateData.interpolator);

    static {
        SHUTTLE_MAP.put(3.22, new ShooterStateData(HoodPositions.HOOD_UP.getPosition(), 18, 0.0));
        SHUTTLE_MAP.put(3.70, new ShooterStateData(HoodPositions.HOOD_UP.getPosition(), 21, 0.0));
        SHUTTLE_MAP.put(4.02, new ShooterStateData(HoodPositions.HOOD_UP.getPosition(), 24, 0.0));
        SHUTTLE_MAP.put(5.56, new ShooterStateData(HoodPositions.HOOD_UP.getPosition(), 30, 0.0));
        SHUTTLE_MAP.put(7.02, new ShooterStateData(HoodPositions.HOOD_UP.getPosition(), 39, 0.0));
        SHUTTLE_MAP.put(8.55, new ShooterStateData(HoodPositions.HOOD_UP.getPosition(), 43, 0.0));
    }
}
