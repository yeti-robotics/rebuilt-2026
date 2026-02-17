package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public class IntakeConfigsAlpha {
    static final int ALPHA_INTAKE_MOTOR_ID = 12;
    static final double OUTTAKE_VOLTAGE = 2;
    static final double INTAKE_VOLTAGE = -2;

    static TalonFXConfiguration ALPHA_TALONFX_CONFIGS = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
}
