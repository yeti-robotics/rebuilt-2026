package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Robot;

public class ClimberConfigsBeta {
    static final int CLIMBER_MOTOR_ID = 9;
    static final int LINEAR_SERVO_CHANNEL = 1;
    static final int CLIMBER_SENSOR_ID = 101;

    static final double ROTOR_TO_SENSOR = 1.0;
    static final double SENSOR_TO_MECHANISM = 35.0;
    public static final double TEST_CLIMBER_SPEED = 0.2;

    public static final double HEIGHT_TOLERANCE = 0.2;

    static final double GEAR_RATIO = 0;

    private static final Slot0Configs SLOT_0_CONFIGS = Robot.isReal()
            ? new Slot0Configs()
                    .withKP(0)
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

    static final TalonFXConfiguration primaryTalonFXConfigs = new TalonFXConfiguration()
            .withSlot0(SLOT_0_CONFIGS)
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(30)
                    .withMotionMagicCruiseVelocity(15)
                    .withMotionMagicJerk(0))
            .withFeedback(new FeedbackConfigs()
                    .withRotorToSensorRatio(ROTOR_TO_SENSOR)
                    .withSensorToMechanismRatio(SENSOR_TO_MECHANISM))
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake));
}
