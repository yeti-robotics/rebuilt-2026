package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.hood.HoodPositions;
import frc.robot.util.ShooterStateData;

public class ShooterConfigsGamma {
    static final int FIRST_SHOOTER_ID = 54;
    static final int SECOND_SHOOTER_ID = 55;
    static final int THIRD_SHOOTER_ID = 56;
    public static final double TEST_SHOOTER_SPEED = 0.6;

    static final double ROTOR_TO_SENSOR = 1;
    static final double SENSOR_TO_MECHANISM = 1;

    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs()
            .withKP(8)
            .withKI(0)
            .withKD(0)
            .withKS(4)
            .withKV(0.55)
            .withKA(256);

    public static final Slot1Configs SLOT_1_CONFIGS = new Slot1Configs()
            .withKP(7.9)
            .withKI(1)
            .withKD(0)
            .withKS(5)
            .withKV(1.1)
            .withKA(256);

    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(1)
            .withMotionMagicAcceleration(256)
            .withMotionMagicJerk(0);

    static final TalonFXConfiguration TOP_MOTOR_CONFIGS = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(SENSOR_TO_MECHANISM)
                    .withRotorToSensorRatio(ROTOR_TO_SENSOR))
            .withSlot0(SLOT_0_CONFIGS)
            .withSlot1(SLOT_1_CONFIGS)
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
        SHOOTER_MAP.put(1.928, new ShooterStateData(HoodPositions.STOW.getPosition(), 24, 0.0));
        SHOOTER_MAP.put(2.321, new ShooterStateData(Units.Rotations.of(0.07), 25, 0.0));
        SHOOTER_MAP.put(2.419, new ShooterStateData(Units.Rotations.of(0.09), 25.5, 0.0));
        SHOOTER_MAP.put(2.615, new ShooterStateData(Units.Rotations.of(0.1), 27, 0.0));
        SHOOTER_MAP.put(2.711, new ShooterStateData(Units.Rotations.of(0.14), 27, 0.0));
        SHOOTER_MAP.put(2.882, new ShooterStateData(Units.Rotations.of(0.2), 26.7, 0.0));
        SHOOTER_MAP.put(3.102, new ShooterStateData(Units.Rotations.of(0.3), 27.5, 0.0));
        SHOOTER_MAP.put(3.363, new ShooterStateData(Units.Rotations.of(0.4), 27.7, 0.0));
        SHOOTER_MAP.put(3.902, new ShooterStateData(Units.Rotations.of(0.5), 28.5, 0.0));
    }

    public static final InterpolatingTreeMap<Double, ShooterStateData> SHUTTLE_MAP =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterStateData.interpolator);

    static {
        SHUTTLE_MAP.put(0.0, new ShooterStateData(HoodPositions.HOOD_UP.getPosition(), 0, 0.0));
    }
}
