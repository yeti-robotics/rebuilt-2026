package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.constants.FieldConstants;
import java.util.function.DoubleSupplier;

public class AutoAim extends Command {

    private final Drive drivetrain;
    private final DoubleSupplier xVelSupplier;
    private final DoubleSupplier yVelSupplier;
    private final TurnToPoint turnToPoint;
    private double currentTag;

    public AutoAim(Drive drivetrain, DoubleSupplier xVelSupplier, DoubleSupplier yVelSupplier) {
        this.drivetrain = drivetrain;
        this.xVelSupplier = xVelSupplier;
        this.yVelSupplier = yVelSupplier;

        turnToPoint = new TurnToPoint();

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

        Translation2d speakerCenter = AllianceFlipUtil.apply(FieldConstants.Hub.centerHubOpening.toTranslation2d());
        turnToPoint.setPointToFace(speakerCenter);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();

        double omega = turnToPoint.calculateAngularVelocity(currentPose);

        drivetrain.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xVelSupplier.getAsDouble() * 1.5,
                        yVelSupplier.getAsDouble() * 1.5,
                        omega,
                        drivetrain.getRotation()
                )
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.runVelocity(new ChassisSpeeds());
    }
}
