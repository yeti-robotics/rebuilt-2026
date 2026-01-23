package frc.robot.subsystems.linslide;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.Robot;

public class LinSlideConfigsAlpha {
    static final int LIN_SLIDE_MOTOR_ID = 60;
    static final int LIN_SLIDE_CANCODER_ID = 61;
    static final double GEAR_RATIO = 3;

    private static final Slot0Configs SLOT_0_CONFIGS = Robot.isReal()
            ? new Slot0Configs()
                    .withKP(1) // placeholder values
                    .withKI(0)
                    .withKD(0)
                    .withKG(1)
                    .withKV(1)
                    .withKA(1)
                    .withKS(1)
                    .withGravityType(GravityTypeValue.Elevator_Static)
            : new Slot0Configs()
            .withKP(124)
            .withKI(0)
            .withKD(64)
            .withKG(0)
            .withKV(0)
            .withKA(0)
            .withKS(0.2)
            .withGravityType(GravityTypeValue.Elevator_Static);

    static final TalonFXConfiguration linSlideTalonFXConfigs = new TalonFXConfiguration()
            .withSlot0(SLOT_0_CONFIGS)
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(1) // placeholder values
                    .withMotionMagicCruiseVelocity(1)
                    .withMotionMagicJerk(0))
            .withFeedback(new FeedbackConfigs().withRotorToSensorRatio(1).withSensorToMechanismRatio(GEAR_RATIO))
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake));

    static final CANcoderConfiguration linSlideCANCoderConfigs = new CANcoderConfiguration()
            .withMagnetSensor(
                    new MagnetSensorConfigs()
                            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                            .withMagnetOffset(0.67) // placeholder value
                    );
}
