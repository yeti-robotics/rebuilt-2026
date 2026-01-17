package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.DoubleSupplier;

public class AutoAimCommands {
    // TODO: tune
    private static final PIDController headingController = new PIDController(5, 0, 0);

    private static double calculateAngularVelocity(Pose2d currentPose, Translation2d target) {
        if (target == null) {
            return 0; // TODO: make target nonnull
        }

        Rotation2d currentHeading = currentPose.getRotation();
        Rotation2d desiredHeading = target.minus(currentPose.getTranslation()).getAngle();

        return headingController.calculate(currentHeading.getRadians(), desiredHeading.getRadians());
    }

    public static Command autoAim(
            Drive drive, DoubleSupplier xVelSupplier, DoubleSupplier yVelSupplier, Translation2d target) {
        Translation2d modifiedTarget = AllianceFlipUtil.apply(target);

        return drive.runEnd(
                () -> {
                    Pose2d currentPose = drive.getPose();

                    double angularVelo = calculateAngularVelocity(currentPose, modifiedTarget);

                    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                            xVelSupplier.getAsDouble() * 1.5,
                            yVelSupplier.getAsDouble() * 1.5,
                            angularVelo,
                            drive.getRotation()));
                },
                drive::stop);
    }
}
