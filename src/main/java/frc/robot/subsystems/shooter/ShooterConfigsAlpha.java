package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.robot.util.ShooterStateData;

public class ShooterConfigsAlpha {
    static final int RIGHT_SHOOTER_ID = 19;
    static final int LEFT_SHOOTER_ID = 0;
    public static final double TEST_SHOOTER_SPEED = 0.8;

    public static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs()
            .withKP(2)
            .withKI(0)
            .withKD(0.5)
            .withKA(1.1)
            .withKV(0.4)
            .withKS(11.5);

    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
            .withMotionMagicAcceleration(8)
            .withMotionMagicCruiseVelocity(4)
            .withMotionMagicJerk(0);

    static final TalonFXConfiguration TOP_MOTOR_CONFIGS = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(2.89).withRotorToSensorRatio(1))
            .withSlot0(SLOT_0_CONFIGS)
            .withMotionMagic(MOTION_MAGIC_CONFIGS);

    static final TalonFXConfiguration BOTTOM_MOTOR_CONFIGS = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(2.89).withRotorToSensorRatio(1))
            .withSlot0(SLOT_0_CONFIGS)
            .withMotionMagic(MOTION_MAGIC_CONFIGS);

    static InterpolatingTreeMap<Double, ShooterStateData> SHOOTER_MAP() {
        InterpolatingTreeMap<Double, ShooterStateData> map =
                new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterStateData.interpolator);
        map.put(0.0, new ShooterStateData(0, 0, 0));

        return map;
    }
}
