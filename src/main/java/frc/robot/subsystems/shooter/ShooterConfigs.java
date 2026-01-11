package frc.robot.subsystems.shooter;

import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterConfigs {


    static final int shooterMotorID = 6;
    static final int shooterMotorCurrentLimit = 60;



    static final SparkMaxConfig shooterConfig =
            (SparkMaxConfig) new SparkMaxConfig()
                    .inverted(true)
                    .smartCurrentLimit(shooterMotorCurrentLimit);
}
