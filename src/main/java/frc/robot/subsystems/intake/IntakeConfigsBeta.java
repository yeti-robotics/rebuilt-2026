package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public class IntakeConfigsBeta {

    static final int OUTER_INTAKE_MOTOR_ID = 13;
    static final int INNER_INTAKE_MOTOR_ID = 12;
    static final double INTAKE_VOLTAGE = 1;
    static final double OUTTAKE_VOLTAGE = -1;

    public static double ROLL_IN_SPEED = 1;
    public static double ROLL_IN_SLOWER = 0.5;

    public static double OUTER_ROLLER_SPEED = -0.7;
    public static double INNER_ROLLER_SPEED = -0.5;

    static TalonFXConfiguration PRIMARY_TALONFX_CONFIGS = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    static TalonFXConfiguration SECONDARY_TALONFX_CONFIGS = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
}
