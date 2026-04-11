package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConfigsGamma;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ShooterStateData;
import org.littletonrobotics.junction.Logger;

public class ReadyAimCommand extends Command {
    private CommandSwerveDrivetrain drive;
    private Shooter shooter;
    private Hood hood;
    private Translation2d target;
    double targetRPS;
    Angle targetHood;
    ShooterStateData state = ShooterConfigsGamma.RED_SHOOTER_MAP.get(0.0);

    public ReadyAimCommand(CommandSwerveDrivetrain drive, Shooter shooter, Hood hood, Translation2d target) {
        this.drive = drive;
        this.shooter = shooter;
        this.target = target;
        this.hood = hood;
    }

    @Override
    public void initialize() {
        Pose2d currentPose = drive.getState().Pose;
        Translation2d modifiedTarget = AllianceFlipUtil.apply(target);
        Translation2d currentPosition = currentPose.getTranslation();
        double distance = modifiedTarget.getDistance(currentPosition);

        state = ShooterConfigsGamma.RED_SHOOTER_MAP.get(distance);  

        targetRPS = state.rps;
        targetHood = state.hoodPos;

        Logger.recordOutput("AutoAimCommands/Shooter Map/Trench Shot", false);
        if ((AllianceFlipUtil.apply(currentPose.getX()) >= 3.600)
                && ((currentPose.getY() >= 7.180 && currentPose.getY() <= 7.527)
                        || (currentPose.getY() <= 1.089 && currentPose.getY() >= 0.469))) {
            targetRPS = 30;
            targetHood = Units.Rotations.of(0.15);
            Logger.recordOutput("AutoAimCommands/Shooter Map/Trench Shot", true);
        }
        Logger.recordOutput("AutoAimCommands/Shooter Map/Target RPS", targetRPS);
        Logger.recordOutput("AutoAimCommands/Shooter Map/Target Hood", targetHood);
    }

    @Override
    public void execute() {
        shooter.spinMotors(targetRPS);
        hood.moveTo(targetHood);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopMotors();
    }
}
