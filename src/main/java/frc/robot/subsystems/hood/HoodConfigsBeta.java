package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.Robot;

public class HoodConfigsBeta {
    static final int HOOD_MOTOR_ID = 44;
    static final int HOOD_CANCODER_ID = 45;
    static final double MAGNET_OFFSET = 0.337646;

    static final double ROTOR_TO_SENSOR = 16;
    static final double SENSOR_TO_MECHANISM = 1.5;

    public static final double TEST_HOOD_SPEED = 0.2;

    private static final Slot0Configs SLOT_0_CONFIGS = Robot.isReal()
            ? new Slot0Configs()
                    .withKP(1) // placeholder values
                    .withKI(0)
                    .withKD(0)
                    .withKG(0)
                    .withKV(0)
                    .withKA(0)
                    .withKS(0)
                    .withGravityType(GravityTypeValue.Arm_Cosine)
            : new Slot0Configs()
                    .withKP(0)
                    .withKI(0)
                    .withKD(0)
                    .withKG(0)
                    .withKV(0)
                    .withKA(0)
                    .withKS(0)
                    .withGravityType(GravityTypeValue.Arm_Cosine);

    static final TalonFXConfiguration HOOD_MOTOR_CONFIGS = new TalonFXConfiguration()
            .withSlot0(SLOT_0_CONFIGS)
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(1) // placeholder values
                    .withMotionMagicCruiseVelocity(1)
                    .withMotionMagicJerk(0))
            .withFeedback(new FeedbackConfigs().withRotorToSensorRatio(ROTOR_TO_SENSOR).withSensorToMechanismRatio(SENSOR_TO_MECHANISM))
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake));

    static final CANcoderConfiguration HOOD_CANCODER_CONFIGS = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                    .withMagnetOffset(MAGNET_OFFSET)
                    .withAbsoluteSensorDiscontinuityPoint(0.625));
}
