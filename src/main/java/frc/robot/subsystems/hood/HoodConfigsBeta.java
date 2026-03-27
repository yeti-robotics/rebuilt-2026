package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import frc.robot.Robot;

public class HoodConfigsBeta {
    static final int HOOD_MOTOR_ID = 44;
    static final int HOOD_CANCODER_ID = 45;
    static final double MAGNET_OFFSET = 0.964355;

    static final double ROTOR_TO_SENSOR = 1;
    static final double SENSOR_TO_MECHANISM = 20;

    public static final double TEST_HOOD_SPEED = 0.2;

    private static final Slot0Configs SLOT_0_CONFIGS = Robot.isReal()
            ? new Slot0Configs()
                    .withKP(32)
                    .withKI(0)
                    .withKD(0)
                    .withKG(0)
                    .withKV(0)
                    .withKA(0)
                    .withKS(0)
                    .withGravityType(GravityTypeValue.Elevator_Static)
            : new Slot0Configs()
                    .withKP(0)
                    .withKI(0)
                    .withKD(0)
                    .withKG(0)
                    .withKV(0)
                    .withKA(0)
                    .withKS(0)
                    .withGravityType(GravityTypeValue.Elevator_Static);

    static final TalonFXConfiguration HOOD_MOTOR_CONFIGS = new TalonFXConfiguration()
            .withSlot0(SLOT_0_CONFIGS)
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(8)
                    .withMotionMagicCruiseVelocity(2)
                    .withMotionMagicJerk(0))
            .withFeedback(new FeedbackConfigs()
                    .withRotorToSensorRatio(ROTOR_TO_SENSOR)
                    .withSensorToMechanismRatio(SENSOR_TO_MECHANISM))
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                    .withReverseSoftLimitThreshold(0.015)
                    .withReverseSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(0.751006)
                    .withForwardSoftLimitEnable(true));

    static final CANcoderConfiguration HOOD_CANCODER_CONFIGS = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                    .withMagnetOffset(MAGNET_OFFSET)
                    .withAbsoluteSensorDiscontinuityPoint(0));
}
