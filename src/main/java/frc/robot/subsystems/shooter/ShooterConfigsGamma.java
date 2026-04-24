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
            .withKP(10)
            .withKI(0)
            .withKD(0)
            .withKS(5)
            .withKV(0.5)
            .withKA(125);

    public static final Slot1Configs SLOT_1_CONFIGS = new Slot1Configs()
            .withKP(6)
            .withKI(1)
            .withKD(0)
            .withKS(5)
            .withKV(1.4)
            .withKA(125);

    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(1)
            .withMotionMagicAcceleration(200)
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

    public static final InterpolatingTreeMap<Double, ShooterStateData> RED_SHOOTER_MAP =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterStateData.interpolator);

    static {
        RED_SHOOTER_MAP.put(1.945, new ShooterStateData(HoodPositions.STOW.getPosition(), 23.5, 0.0));
        RED_SHOOTER_MAP.put(2.199, new ShooterStateData(HoodPositions.STOW.getPosition(), 25.5, 0.0));
        RED_SHOOTER_MAP.put(2.222, new ShooterStateData(HoodPositions.STOW.getPosition(), 25.75, 0.0));
        RED_SHOOTER_MAP.put(2.354, new ShooterStateData(HoodPositions.STOW.getPosition(), 26, 0.0));
        RED_SHOOTER_MAP.put(2.442, new ShooterStateData(HoodPositions.STOW.getPosition(), 26.5, 0.0));
        RED_SHOOTER_MAP.put(2.522, new ShooterStateData(Units.Rotations.of(0.1), 27, 0.0));
        RED_SHOOTER_MAP.put(2.657, new ShooterStateData(Units.Rotations.of(0.12), 27.3, 0.0));
        RED_SHOOTER_MAP.put(2.706, new ShooterStateData(Units.Rotations.of(0.149), 27, 0.0));
        RED_SHOOTER_MAP.put(2.882, new ShooterStateData(Units.Rotations.of(0.3), 26.8, 0.0));
        RED_SHOOTER_MAP.put(3.105, new ShooterStateData(Units.Rotations.of(0.280), 28, 0.0));
        RED_SHOOTER_MAP.put(3.362, new ShooterStateData(Units.Rotations.of(0.3), 28.5, 0.0));
    }

    public static final InterpolatingTreeMap<Double, ShooterStateData> BLUE_SHOOTER_MAP =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterStateData.interpolator);

    static {
        BLUE_SHOOTER_MAP.put(1.945, new ShooterStateData(HoodPositions.STOW.getPosition(), 25, 0.0));
        BLUE_SHOOTER_MAP.put(2.199, new ShooterStateData(HoodPositions.STOW.getPosition(), 27, 0.0));
        BLUE_SHOOTER_MAP.put(2.354, new ShooterStateData(HoodPositions.STOW.getPosition(), 26, 0.0));
        BLUE_SHOOTER_MAP.put(2.442, new ShooterStateData(HoodPositions.STOW.getPosition(), 28, 0.0));
        BLUE_SHOOTER_MAP.put(2.522, new ShooterStateData(Units.Rotations.of(0.1), 28.5, 0.0));
        BLUE_SHOOTER_MAP.put(2.627, new ShooterStateData(Units.Rotations.of(0.177), 28, 0.0));
        BLUE_SHOOTER_MAP.put(2.838, new ShooterStateData(Units.Rotations.of(0.249), 31, 0.0));
        BLUE_SHOOTER_MAP.put(3.007, new ShooterStateData(Units.Rotations.of(0.2), 30.5, 0.0));
        BLUE_SHOOTER_MAP.put(3.195, new ShooterStateData(Units.Rotations.of(0.34), 32, 0.0));
        BLUE_SHOOTER_MAP.put(3.362, new ShooterStateData(Units.Rotations.of(0.3), 29, 0.0));
    }

    public static final InterpolatingTreeMap<Double, ShooterStateData> SHUTTLE_MAP =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterStateData.interpolator);

    static {
        SHUTTLE_MAP.put(0.0, new ShooterStateData(HoodPositions.HOOD_UP.getPosition(), 0, 0.0));
    }
}
