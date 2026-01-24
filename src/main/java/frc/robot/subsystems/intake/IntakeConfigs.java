package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public class IntakeConfigs {
    static final int INTAKE_MOTOR_ID = 33;

    public static final int INTAKE_ROLL_IN_VOLTAGE = 0;
    public static final int INTAKE_ROLL_OUT_VOLTAGE = 0;

    static TalonFXConfiguration INTAKE_TALONFX_CONFIGS = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
}
