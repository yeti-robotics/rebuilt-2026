package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConfigsBeta;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ShooterStateData;
import org.littletonrobotics.junction.Logger;

public class ReadyAimCommand extends Command {
    private CommandSwerveDrivetrain drive;
    private Shooter shooter;
    private Translation2d target;

    public ReadyAimCommand(CommandSwerveDrivetrain drive, Shooter shooter, Translation2d target) {
        this.drive = drive;
        this.shooter = shooter;
        this.target = target;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        Pose2d currentPose = drive.getState().Pose;
        Translation2d modifiedTarget = AllianceFlipUtil.apply(target);
        Translation2d currentPosition = currentPose.getTranslation();
        double distance = modifiedTarget.getDistance(currentPosition);

        ShooterStateData state = ShooterConfigsBeta.SHOOTER_MAP.get(distance);

        double targetRPS = state.rps;

        Logger.recordOutput("AutoAimCommands/Shooter Map/target rps", targetRPS);

        shooter.spinMotors(targetRPS);

        Translation2d difference = modifiedTarget.minus(currentPosition);
        double targetAngle = Math.atan2(difference.getY(), difference.getX());
        double currentAngle = currentPose.getRotation().getRadians();

        double angleError = MathUtil.angleModulus(targetAngle - currentAngle);
        double angleErrorDeg = Math.toDegrees(Math.abs(angleError));
        Logger.recordOutput("AutoAimCommands/Angle Error Deg", angleErrorDeg);

        if (angleErrorDeg <= 5) {
            drive.applyRequest(SwerveRequest.SwerveDriveBrake::new);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopMotors();
    }
}
