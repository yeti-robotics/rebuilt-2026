package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeRoller;

    public IntakeSubsystem() {
        intakeRoller = new SparkMax(IntakeConfigs.intakeMotorID, SparkLowLevel.MotorType.kBrushed);
        intakeRoller.configure(IntakeConfigs.feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command setVoltage(double voltage) {
        return runEnd(
                () -> {
                    intakeRoller.setVoltage(voltage);
                },
                () -> {
                    intakeRoller.setVoltage(0);
                }
        );

    }

}


