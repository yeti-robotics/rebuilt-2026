package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.*;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LimelightHelpers;
import java.util.function.DoubleSupplier;

public class AutoAim extends Command {

    private final Drive drivetrain;
    private final DoubleSupplier xVelSupplier;
    private final TurnToPoint poseAimRequest;
    private final DoubleSupplier yVelSupplier;
    private double currentTag;

    public AutoAim(Drive drivetrain, DoubleSupplier xVelSupplier, DoubleSupplier yVelSupplier) {

        this.drivetrain = drivetrain;
        this.xVelSupplier = xVelSupplier;
        this.yVelSupplier = yVelSupplier;

        addRequirements(drivetrain);

        poseAimRequest = new TurnToPoint();
        poseAimRequest.HeadingController.setPID(5, 0, 0);
        poseAimRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        currentTag = LimelightHelpers.getFiducialID(VisionConstants.camera0Name);

        Translation2d speakerCenter = AllianceFlipUtil.apply(FieldConstants.Hub.centerHubOpening.toTranslation2d());

        poseAimRequest.setPointToFace(speakerCenter);
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getFiducialID(
                        VisionConstants.camera0Name.isBlank()
                                ? VisionConstants.camera0Name
                                : VisionConstants.camera1Name)
                == currentTag) {
            drivetrain.run(() -> poseAimRequest
                    .withVelocityX(xVelSupplier.getAsDouble() * 1.5)
                    .withVelocityY(yVelSupplier.getAsDouble() * 1.5));
        } else {
            end(true);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
