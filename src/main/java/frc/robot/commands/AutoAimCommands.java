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
    private static final PIDController headingController = new PIDController(2, 0, 0);
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
                    System.out.println("Cmd1");
                    Pose2d currentPose = drive.getPose();
                    Translation2d currentPosition = currentPose.getTranslation();
                    Rotation2d currentRotation = drive.getRotation();

                    Optional<Rotation2d> visionAngle = vision.getTargetAngle(cameraIndex);
                    if (visionAngle.isPresent()) {
                        Translation2d toTarget = modifiedTarget.minus(currentPosition);
                        double currentDistance = toTarget.getNorm();
                        Rotation2d targetAngle = new Rotation2d(toTarget.getX(), toTarget.getY());

                        double xSpeed = -xVelSupplier.getAsDouble();
                        double ySpeed = yVelSupplier.getAsDouble();
                        double tangentSpeed = Math.hypot(xSpeed, ySpeed);

                        if (tangentSpeed > MIN_MOVEMENT_THRESHOLD) {
                            double joystickAngle = Math.atan2(ySpeed, xSpeed);
                            double joystickMagnitude = Math.hypot(xSpeed, ySpeed);

                            // Calculate orbital and radial movement components
                            double tangentComponent = Math.sin(joystickAngle) * joystickMagnitude;
                            Rotation2d tangentDirection = targetAngle.plus(Rotation2d.fromRadians(Math.PI / 2));

                            double radialComponent = Math.cos(joystickAngle) * joystickMagnitude;
                            Rotation2d radialDirection = targetAngle;

                            double tangentX = tangentDirection.getCos() * tangentComponent;
                            double tangentY = tangentDirection.getSin() * tangentComponent;
                            double radialX = -radialDirection.getCos() * radialComponent;
                            double radialY = -radialDirection.getSin() * radialComponent;

                            double fieldX = tangentX + radialX;
                            double fieldY = tangentY + radialY;

                            double rotationSpeed = headingController.calculate(
                                    currentRotation.getRadians(),
                                    targetAngle.getRadians()
                                            + Math.PI); // remove pi if needed to rotate the robot the right direction

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

    public static Command autoAimWithOrbitModified(
            Drive drive,
            Vision vision,
            int cameraIndex,
            DoubleSupplier xVelSupplier,
            DoubleSupplier yVelSupplier,
            Translation2d target) {

        Translation2d modifiedTarget = AllianceFlipUtil.apply(target);
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        return drive.runEnd(
                () -> {
                    System.out.println("Cmd2");
                    Pose2d currentPose = drive.getPose();
                    Translation2d currentPosition = currentPose.getTranslation();
                    Rotation2d currentRotation = currentPose.getRotation();

                    Translation2d hubDistance = modifiedTarget.minus(currentPosition);


//
//                    double theta = hubDistance.getAngle().getRadians();
//
//                    System.out.println(hubDistance.getAngle().getDegrees());
//
//                    theta += Math.PI; // b/c we want robot back to be facing



                    double rawXVelo = xVelSupplier.getAsDouble(); // because field relativity
                    double rawYVelo = yVelSupplier.getAsDouble();

                    Rotation2d targetHeading = hubDistance.getAngle().plus(Rotation2d.kPi);
                    Translation2d fieldRel = new Translation2d(rawXVelo, rawYVelo).rotateBy(targetHeading);

//                    double thetaCos = Math.cos(theta);
//                    double thetaSin = Math.sin(theta);
//
//                    // Rotation Matrix
//                    double xVeloRotated = rawXVelo * thetaCos - rawYVelo * thetaSin;
//                    double yVeloRotated = rawXVelo * thetaCos + rawYVelo * thetaCos;

                    double angularVelo = headingController.calculate(
                            currentRotation.getRadians(),
                            targetHeading.getRadians());

                    ChassisSpeeds currentReference = ChassisSpeeds.fromFieldRelativeSpeeds(
                            fieldRel.getX(),
                            fieldRel.getY(),
                            angularVelo,
                            currentRotation);

                    drive.runVelocity(currentReference);
                },
                drive::stop);
    }
}
