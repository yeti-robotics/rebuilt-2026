package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public class IntakeConfigs {
    static final int PRIMARY_INTAKE_MOTOR_ID = 12;
    static final int SECONDARY_INTAKE_MOTOR_ID = 13;
    static final double INTAKE_VOLTAGE = 1;
    static final double OUTTAKE_VOLTAGE = -1;

    static TalonFXConfiguration PRIMARY_TALONFX_CONFIGS = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    static TalonFXConfiguration SECONDARY_TALONFX_CONFIGS = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
}
