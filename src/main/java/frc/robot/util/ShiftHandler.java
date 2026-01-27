package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Utility class to handle the alliance shift logic */
public class ShiftHandler extends SubsystemBase {
    public enum ShiftType {
        TRANSITION_SHIFT,
        RED_SHIFT,
        BLUE_SHIFT,
        ENDGAME_SHIFT,
        UNKNOWN;

        private ShiftType invert() {
            return (this == RED_SHIFT || this == BLUE_SHIFT) ? (this == RED_SHIFT ? BLUE_SHIFT : RED_SHIFT) : this;
        }
    }

    private ShiftType firstShift;
    private boolean teleopDataRead = false;
    private ShiftType currentShift = null;

    public ShiftHandler() {}

    @Override
    public void periodic() {
        if (DriverStation.isTeleopEnabled()) {
            double matchTime = DriverStation.getMatchTime();
            if (!teleopDataRead) {
                String gameData = DriverStation.getGameSpecificMessage();

                if (!gameData.isEmpty()) {
                    switch (gameData.charAt(0)) {
                        case 'B':
                            firstShift = ShiftType.RED_SHIFT;
                            teleopDataRead = true;
                            break;
                        case 'R':
                            firstShift = ShiftType.BLUE_SHIFT;
                            teleopDataRead = true;
                            break;
                        default:
                            Logger.recordOutput("Current Shift", "NOT FOUND");
                            firstShift = ShiftType.UNKNOWN;
                            break;
                    }
                }
            }

            if (matchTime >= 130) {
                currentShift = ShiftType.TRANSITION_SHIFT;
            } else if (matchTime <= 30) {
                currentShift = ShiftType.ENDGAME_SHIFT;
            } else {
                int blockTime = (int) (130 - matchTime) / 25;
                currentShift = blockTime % 2 == 0 ? firstShift : firstShift.invert();
            }
        }
        if (currentShift != null) {
            Logger.recordOutput("Current Shift", currentShift.toString());
        }
    }

    public ShiftType getCurrentShift() {
        return currentShift;
    }
}
