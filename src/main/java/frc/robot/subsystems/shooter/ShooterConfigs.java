package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ShooterConfigs {
    static final int topMotorID = 5;
    static final int bottomMotorID = 6;

    // TOP IS TWO TO ONE BOTTOM IS ONE TO ONE
    static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(2));
}
