package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConfigsBeta;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ShooterStateData;
import org.littletonrobotics.junction.Logger;

public class ShuttleReadyAimCommand  extends Command {
    private CommandSwerveDrivetrain drive;
    private Shooter shooter;
    private Hood hood;

    public ShuttleReadyAimCommand(CommandSwerveDrivetrain drive, Shooter shooter, Hood hood) {
        this.drive = drive;
        this.shooter = shooter;
        this.hood = hood;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        Pose2d currentPose = drive.getState().Pose;
        Translation2d modifiedTarget = new Translation2d(AllianceFlipUtil.apply(2.35), currentPose.getY());
        Translation2d currentPosition = currentPose.getTranslation();
        double distance = modifiedTarget.getDistance(currentPosition);

        ShooterStateData state = ShooterConfigsBeta.SHUTTLE_MAP.get(distance);

        double targetRPS = state.rps;
        Angle targetHoodPos = state.hoodPos;

        Logger.recordOutput("AutoAimCommands/Shuttle Map/target rps", targetRPS);
        Logger.recordOutput("AutoAimCommands/Shuttle Map/target hood position", targetHoodPos);

        shooter.shoot(targetRPS);
        hood.hoodPosition(targetHoodPos.magnitude());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopMotors();
    }
}

