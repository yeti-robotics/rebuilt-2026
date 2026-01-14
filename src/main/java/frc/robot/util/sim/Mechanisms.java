package frc.robot.util.sim;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

@Logged
public class Mechanisms {
    public Mechanism2d elevatorMech;
    private final MechanismLigament2d liftLigament;

    private final StructArrayPublisher<Pose3d> realComponentPosePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("ComponentPoses/Real", Pose3d.struct)
            .publish();
    private final StructArrayPublisher<Pose3d> targetComponentPosePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("ComponentPoses/Target", Pose3d.struct)
            .publish();

    public Mechanisms() {
        elevatorMech = new Mechanism2d(Units.inchesToMeters(60), Units.inchesToMeters(100));

        liftLigament = elevatorMech
                .getRoot("startPoint", Units.inchesToMeters(30), Units.inchesToMeters(4))
                .append(new MechanismLigament2d("lift", Units.feetToMeters(3), 90, 6, new Color8Bit(Color.kRed)));
        elevatorMech
                .getRoot("startPoint", Units.inchesToMeters(30), Units.inchesToMeters(4))
                .append(new MechanismLigament2d("bottom", Units.feetToMeters(3), 0, 6, new Color8Bit(Color.kGreen)));
    }

    public void updateElevatorMech(double elevatorPos) {
        liftLigament.setLength(Units.inchesToMeters((elevatorPos * 6) + 1));
        SmartDashboard.putData("Mechanisms/CoralManipulator", elevatorMech);
    }

    public void publishComponentPoses(double elevatorPos, boolean useRealPoses) {
        double elevatorStageHeight = Units.inchesToMeters(elevatorPos * 8.6);
        double carriageHeight = Units.inchesToMeters(elevatorPos * 15);

        (useRealPoses ? realComponentPosePublisher : targetComponentPosePublisher).set(new Pose3d[] {
            new Pose3d(
                    Units.inchesToMeters(-8), 0.0, Units.inchesToMeters(2.625) + elevatorStageHeight, Rotation3d.kZero),
            new Pose3d(Units.inchesToMeters(-4.13), 0, Units.inchesToMeters(10.22) + carriageHeight, Rotation3d.kZero),
        });
    }
}
