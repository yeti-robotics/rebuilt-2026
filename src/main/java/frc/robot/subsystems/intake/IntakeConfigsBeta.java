package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public class IntakeConfigsBeta {

    static final int RIGHT_INTAKE_MOTOR_ID = 13;
    static final int LEFT_INTAKE_MOTOR_ID = 12;
    static final double INTAKE_VOLTAGE = 1;
    static final double OUTTAKE_VOLTAGE = -1;

    public static double ROLL_IN_SPEED = 1;
    public static double ROLL_IN_SLOWER = 0.5;

    public static double ROLLER_SPEED = -1;
    public static double INNER_ROLLER_SPEED = -0.5;

    static TalonFXConfiguration RIGHT_TALONFX_CONFIGS = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(40)
                    .withSupplyCurrentLimit(70)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLowerLimit(40)
                    .withSupplyCurrentLowerTime(1));
}
