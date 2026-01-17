package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static final double FIELD_LENGTH = 0;

    public static final class Hub {
        public static Translation3d farLeftHub =
                new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0));
        public static Translation3d closeRightHub =
                new Translation3d(0.0, Units.inchesToMeters(0), Units.inchesToMeters(0));
        public static Translation3d centerHubOpening = farLeftHub.interpolate(closeRightHub, 0.5);
        public static final AprilTagFieldLayout aprilTags;

        static {
            aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        }
    }

    public static final class Shuttle {
        public static Translation2d closeShuttleTargetCorner = new Translation2d(0.0, 0.0);

        public static Translation2d farShuttleTargetCorner = new Translation2d(0.0, 0.0);

        public static Translation2d shuttleTargetZone =
                closeShuttleTargetCorner.interpolate(farShuttleTargetCorner, 0.5);
    }
}
