package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TurnToPoint {

    private final PIDController headingController;

    private Translation2d pointToFace;

    public TurnToPoint() {
        headingController = new PIDController(5.0, 0.0, 0.0);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setPointToFace(Translation2d point) {
        this.pointToFace = point;
    }

    public double calculateAngularVelocity(Pose2d currentPose) {
        if (pointToFace == null) {
            return 0;
        }
        Rotation2d currentHeading = currentPose.getRotation();
        Rotation2d desiredHeading = pointToFace.minus(currentPose.getTranslation()).getAngle();

        return headingController.calculate(currentHeading.getRadians(), desiredHeading.getRadians());
    }
}
