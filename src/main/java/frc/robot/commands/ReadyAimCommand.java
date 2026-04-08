package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    ShooterStateData state = ShooterConfigsGamma.SHOOTER_MAP.get(0.0);

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

        state = ShooterConfigsGamma.SHOOTER_MAP.get(distance);

        targetRPS = state.rps;

        Logger.recordOutput("AutoAimCommands/Shooter Map/target rps", targetRPS);
    }

    @Override
    public void execute() {
        shooter.spinMotors(targetRPS);
        hood.moveTo(state.hoodPos);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopMotors();
    }
}
