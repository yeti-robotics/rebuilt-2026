package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceFlipUtil;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class AutoAimCommands {
    private static final double MIN_MOVEMENT_THRESHOLD = 0.15;
    private static final double CENTRIPETAL_GAIN = 2.0;
    private static final PIDController headingController = new PIDController(5, 0, 0);
    private static final PIDController distanceController = new PIDController(2.0, 0, 0.1);

    public static Command autoAimWithOrbit(
            Drive drive,
            Vision vision,
            int cameraIndex,
            DoubleSupplier xVelSupplier,
            DoubleSupplier yVelSupplier,
            Translation2d target) {

        Translation2d modifiedTarget = AllianceFlipUtil.apply(target);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        distanceController.enableContinuousInput(-Math.PI, Math.PI);

        return drive.runEnd(
                () -> {
                    Pose2d currentPose = drive.getPose();
                    Translation2d currentPosition = currentPose.getTranslation();
                    Rotation2d currentRotation = drive.getRotation();

                    Optional<Rotation2d> visionAngle = vision.getTargetAngle(cameraIndex);
                    if (visionAngle.isPresent()) {
                        Translation2d toTarget = modifiedTarget.minus(currentPosition);
                        double currentDistance = toTarget.getNorm();
                        Rotation2d targetAngle = new Rotation2d(toTarget.getX(), toTarget.getY());

                        double xSpeed = xVelSupplier.getAsDouble();
                        double ySpeed = yVelSupplier.getAsDouble();
                        double tangentSpeed = Math.hypot(xSpeed, ySpeed);

                        if (tangentSpeed > MIN_MOVEMENT_THRESHOLD) {
                            double joystickAngle = Math.atan2(ySpeed, xSpeed);
                            double joystickMagnitude = Math.hypot(xSpeed, ySpeed);

                            Rotation2d tangentDirection = targetAngle.plus(
                                    Rotation2d.fromRadians(Math.signum(Math.sin(joystickAngle)) * Math.PI / 2));

                            double tangentX = joystickMagnitude * tangentDirection.getCos();
                            double tangentY = joystickMagnitude * tangentDirection.getSin();

                            double distanceError = currentDistance - toTarget.getNorm();
                            double centripetalX = -distanceError * targetAngle.getCos() * CENTRIPETAL_GAIN;
                            double centripetalY = -distanceError * targetAngle.getSin() * CENTRIPETAL_GAIN;

                            double fieldX = tangentX + centripetalX;
                            double fieldY = tangentY + centripetalY;

                            double rotationSpeed = headingController.calculate(
                                    currentRotation.getRadians(), targetAngle.getRadians() + Math.PI); // remove pi if needed to rotate the robot the right direction

                            double robotVx = fieldX * currentRotation.getCos() + fieldY * currentRotation.getSin();
                            double robotVy = -fieldX * currentRotation.getSin() + fieldY * currentRotation.getCos();

                            drive.runVelocity(new ChassisSpeeds(robotVx, robotVy, rotationSpeed));
                            return;
                        }
                    }

                    double angularVelo = headingController.calculate(
                            currentRotation.getRadians(),
                            modifiedTarget.minus(currentPosition).getAngle().getRadians() + Math.PI);

                    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                            xVelSupplier.getAsDouble() * 1.5,
                            yVelSupplier.getAsDouble() * 1.5,
                            angularVelo,
                            currentRotation));
                },
                drive::stop);
    }
}
