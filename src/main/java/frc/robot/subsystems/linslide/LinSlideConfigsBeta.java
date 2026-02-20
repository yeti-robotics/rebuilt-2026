package frc.robot.subsystems.linslide;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.Robot;

public class LinSlideConfigsBeta {
    static final int LIN_SLIDE_MOTOR_ID = 53;
    static final int LIN_SLIDE_CANCODER_ID = 54;

    static final double ROTOR_TO_SENSOR = 50/12;
    static final double SENSOR_TO_MECHANISM = 24/18;

    static final double MAGNET_OFFSET = 0; // placeholder value

    private static final Slot0Configs SLOT_0_CONFIGS = Robot.isReal()
            ? new Slot0Configs()
                    .withKP(1) // placeholder values
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

    static final TalonFXConfiguration linSlideTalonFXConfigs = new TalonFXConfiguration()
            .withSlot0(SLOT_0_CONFIGS)
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(1) // placeholder values
                    .withMotionMagicCruiseVelocity(1)
                    .withMotionMagicJerk(0))
            .withFeedback(new FeedbackConfigs().withRotorToSensorRatio(ROTOR_TO_SENSOR).withSensorToMechanismRatio(SENSOR_TO_MECHANISM))
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake));

    static final CANcoderConfiguration linSlideCancoderConfiguration = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                    .withMagnetOffset(MAGNET_OFFSET)
                    .withAbsoluteSensorDiscontinuityPoint(0.625));
}
