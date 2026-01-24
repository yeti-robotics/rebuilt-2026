package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class HoodConfigs {
    static final int HOOD_MOTOR_ID = 44;
    static final int HOOD_CANCODER_ID = 45;
    static final TalonFXConfiguration HOOD_MOTOR_CONFIGS = new TalonFXConfiguration();
    static final CANcoderConfiguration HOOD_CANCODER_CONFIGS = new CANcoderConfiguration();
}