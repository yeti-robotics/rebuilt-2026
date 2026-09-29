package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConfigs;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConfigs;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ShooterStateData;
import org.littletonrobotics.junction.Logger;

public class SOTMCommand extends Command {

    private final CommandSwerveDrivetrain drive;
    private final Shooter shooter;
    private final Hood hood;
    private final Turret turret;
    private final Translation2d target;

    public SOTMCommand(CommandSwerveDrivetrain drive, Shooter shooter, Hood hood, Turret turret, Translation2d target) {
        this.drive = drive;
        this.shooter = shooter;
        this.hood = hood;
        this.turret = turret;
        this.target = target;
    }

    private Angle calcDesiredTurretHeading() {
        Pose2d robotPose = drive.getState().Pose;
        Translation2d turretPosition = robotPose
                .transformBy(new Transform2d(TurretConfigs.turretOffset, new Rotation2d()))
                .getTranslation();

        Translation2d targetToTurret = target.minus(turretPosition);
        Rotation2d fieldRelativeAngle = targetToTurret.getAngle();

        Rotation2d turretAngle = fieldRelativeAngle.minus(robotPose.getRotation());

        return turretAngle.getMeasure();
    }

    @Override
    public void execute() {
        Pose2d currentPose = drive.getState().Pose;
        Translation2d modifiedTarget = AllianceFlipUtil.apply(target);
        Translation2d currentPosition = currentPose.getTranslation();
        double distance = modifiedTarget.getDistance(currentPosition);

        ShooterStateData state = ShooterConfigs.SHOOTER_MAP.get(distance);
        double timeOfFlight = state.timeOfFlight;

        ChassisSpeeds speeds = drive.getState().Speeds;

        Translation2d robotVelocity = new Translation2d(speeds.vx, speeds.vy);
        Translation2d robotDisplacement = robotVelocity.times(timeOfFlight);
        Translation2d compensatedTarget = modifiedTarget.minus(robotDisplacement);
        double compensatedDistance = compensatedTarget.getDistance(currentPosition);

        ShooterStateData compensatedState = ShooterConfigs.SHOOTER_MAP.get(compensatedDistance);

        double targetRPS = compensatedState.rps;
        Angle targetHoodAngle = compensatedState.hoodPos;
        Angle targetTurretAngle = calcDesiredTurretHeading();

        Logger.recordOutput("SOTM/Target RPS", targetRPS);
        Logger.recordOutput("SOTM/Target Hood Angle", targetHoodAngle.magnitude());
        Logger.recordOutput("SOTM/Target Turret Angle", targetTurretAngle.magnitude());

        turret.moveTo(targetTurretAngle);
        hood.moveTo(targetHoodAngle);
        shooter.spinAt(targetRPS);
    }
}
