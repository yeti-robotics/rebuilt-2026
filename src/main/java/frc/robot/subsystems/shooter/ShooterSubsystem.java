package frc.robot.subsystems.shooter;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooterMotor;

    public ShooterSubsystem() {
        shooterMotor = new SparkMax(ShooterConfigs.shooterMotorID, MotorType.kBrushed);
        shooterMotor.configure(ShooterConfigs.shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command launchShooter(double voltage) {
        return run(() -> shooterMotor.setVoltage(voltage));
    }
}


