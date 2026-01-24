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
    public static final PIDController headingController = new PIDController(5, 0, 0);

    private static final double SPEED_MULTIPLIER = 1.5;

    static {
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static Command autoAimWithOrbit(
            Drive drive, DoubleSupplier xVelSupplier, DoubleSupplier yVelSupplier, Translation2d target) {
        Translation2d modifiedTarget = AllianceFlipUtil.apply(target);

        return drive.runEnd(
                () -> {
                    Pose2d currentPose = drive.getPose();
                    Translation2d currentPosition = currentPose.getTranslation();
                    Rotation2d currentRotation = currentPose.getRotation();

                    Translation2d hubDistance = modifiedTarget.minus(currentPosition);

                    double rawXVelo = xVelSupplier.getAsDouble() * SPEED_MULTIPLIER;
                    double rawYVelo = yVelSupplier.getAsDouble() * SPEED_MULTIPLIER;

                    Rotation2d targetHeading =
                            hubDistance.getAngle().plus(Rotation2d.kPi); // remove if needed for real robot
                    Translation2d fieldRel = new Translation2d(rawXVelo, rawYVelo).rotateBy(targetHeading);

                    double angularVelo =
                            headingController.calculate(currentRotation.getRadians(), targetHeading.getRadians());

                    ChassisSpeeds currentReference = ChassisSpeeds.fromFieldRelativeSpeeds(
                            fieldRel.getX(), fieldRel.getY(), angularVelo, currentRotation);

                    drive.runVelocity(currentReference);
                },
                drive::stop);
    }

    private static double calculateAngularVelocity(Pose2d currentPose, Translation2d target) {
        if (target == null) {
            return 0;
        }

        Rotation2d currentHeading = currentPose.getRotation();
        Rotation2d desiredHeading = target.minus(currentPose.getTranslation())
                .getAngle()
                .rotateBy(Rotation2d.k180deg); // Remove this .rotateBy() if needed for real bot

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
                            -xVelSupplier.getAsDouble() * SPEED_MULTIPLIER,
                            -yVelSupplier.getAsDouble() * SPEED_MULTIPLIER,
                            angularVelo,
                            drive.getRotation()));
                },
                drive::stop);
    }
}
