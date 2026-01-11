package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeConfigs {
    static final int intakeMotorID = 5;
    static final int intakeMotorCurrentLimit = 60;

    static final SparkMaxConfig feederConfig =
            (SparkMaxConfig) new SparkMaxConfig()
                    .inverted(true)
                    .smartCurrentLimit(intakeMotorCurrentLimit);



}
