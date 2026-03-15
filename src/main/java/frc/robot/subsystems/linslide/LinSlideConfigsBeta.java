package frc.robot.subsystems.linslide;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import frc.robot.Robot;

public class LinSlideConfigsBeta {
    static final int LIN_SLIDE_MOTOR_ID = 53;

    static final double ROTOR_TO_SENSOR = 1;
    static final double SENSOR_TO_MECHANISM = (50.0 / 12.0) * (24.0 / 18.0);

    public static final double DEPLOY_SPEED = 0.35;
    public static final double STOP = 0;

    public static final double LINSLIDE_AUTO_SHOOT_SPEED = -0.1;
    public static final double LINSLIDE_AUTO_STOWING_SPEED = -0.2;

    public static final double LINSLIDE_INTAKE_POSITION = 1.437744;

    private static final Slot0Configs SLOT_0_CONFIGS = Robot.isReal()
            ? new Slot0Configs()
                    .withKP(64)
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
                    .withMotionMagicAcceleration(1)
                    .withMotionMagicCruiseVelocity(1)
                    .withMotionMagicJerk(0))
            .withFeedback(new FeedbackConfigs()
                    .withRotorToSensorRatio(ROTOR_TO_SENSOR)
                    .withSensorToMechanismRatio(SENSOR_TO_MECHANISM))
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                    .withReverseSoftLimitEnable(true)
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(1.5)
                    .withReverseSoftLimitThreshold(0.01))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(50)
                    .withSupplyCurrentLimit(70)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLowerLimit(40)
                    .withSupplyCurrentLowerTime(1));
}
