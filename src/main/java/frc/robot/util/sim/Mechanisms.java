package frc.robot.util.sim;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

@Logged
public class Mechanisms {
    public Mechanism2d climberMechanism;
    private final MechanismLigament2d liftLigament;

    public Mechanisms() {
        climberMechanism = new Mechanism2d(Units.inchesToMeters(60), Units.inchesToMeters(100));

        // fix all values from here down
        liftLigament = climberMechanism
                .getRoot("startPoint", Units.inchesToMeters(30), Units.inchesToMeters(4))
                .append(new MechanismLigament2d("lift", Units.feetToMeters(3), 90, 6, new Color8Bit(Color.kRed)));

        // except this one
        climberMechanism
                .getRoot("startPoint", Units.inchesToMeters(30), Units.inchesToMeters(4))
                .append(new MechanismLigament2d("bottom", Units.feetToMeters(3), 0, 6, new Color8Bit(Color.kGreen)));
    }

    public void updateClimberMechanism(double climberPosition) {
        liftLigament.setLength(Units.inchesToMeters((climberPosition * 6) + 1));
        SmartDashboard.putData("Mechanisms/Climber", climberMechanism);
    }

    public void publishComponentPoses(double climberPosition, boolean useRealPoses) {
        // double climberStageHeight = Units.inchesToMeters(climberPos * 8.6);

        Logger.recordOutput(
                "ComponentPoses/" + (useRealPoses ? "Real" : "Target"),
                new Pose3d(
                        Units.inchesToMeters(-8),
                        0.0,
                        Units.inchesToMeters(2.625) + climberPosition,
                        Rotation3d.kZero));
    }
}
