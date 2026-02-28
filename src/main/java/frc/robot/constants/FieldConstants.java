package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class FieldConstants {
    public static final double FIELD_LENGTH = Units.inchesToMeters(651.2);
    public static final double FIELD_WIDTH = 317.7; // inches

    public static final class Hub {
        public static final double CENTER_HUB_X = 158.6 + 47 / 2; // inches
        public static Translation3d farLeftHubCorner = new Translation3d(
                Units.inchesToMeters(CENTER_HUB_X),
                Units.inchesToMeters((FIELD_WIDTH + 47) / 2),
                Units.inchesToMeters(72));

        public static Translation3d closeRightHubCorner = new Translation3d(
                Units.inchesToMeters(CENTER_HUB_X),
                Units.inchesToMeters((FIELD_WIDTH - 47) / 2),
                Units.inchesToMeters(72));

        public static Translation3d centerHubOpening = farLeftHubCorner.interpolate(closeRightHubCorner, 0.5);

        public static final AprilTagFieldLayout aprilTags;

        static {
            aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            Logger.recordOutput("Center Hub Opening Transform", centerHubOpening);
        }
    }

    public static final class Shuttle {
        public static Translation2d closeShuttleTargetCorner = new Translation2d(0.0, 0.0);

        public static Translation2d farShuttleTargetCorner = new Translation2d(0.0, 0.0);

        public static Translation2d shuttleTargetZone =
                closeShuttleTargetCorner.interpolate(farShuttleTargetCorner, 0.5);
    }
}
