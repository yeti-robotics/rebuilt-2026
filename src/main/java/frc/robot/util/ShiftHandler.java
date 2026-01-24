package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.led.LEDModes;
import org.littletonrobotics.junction.Logger;

/** Utility class to handle the alliance shift logic */
public class ShiftHandler extends SubsystemBase {
    public enum ShiftType {
        TRANSITION_SHIFT("Transition"),
        RED_SHIFT("Red"),
        BLUE_SHIFT("Blue"),
        ENDGAME_SHIFT("Endgame");

        private ShiftType invert() {
            return this == RED_SHIFT ? BLUE_SHIFT : RED_SHIFT;
        }

        public String getShiftString() {
            return shiftString;
        }

        public final String shiftString;

        ShiftType(String shiftString) {
            this.shiftString = shiftString;
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
                            Logger.recordOutput("Shift", "NOT FOUND");
                            firstShift = ShiftType.TRANSITION_SHIFT;
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
    }

    public ShiftType getCurrentShift() {
        return currentShift;
    }

    public static LEDModes mapShiftToLED(ShiftType shift) {
        return switch (shift) {
            case TRANSITION_SHIFT -> LEDModes.TRANSITION_ACTIVE;
            case RED_SHIFT -> LEDModes.RED_ALLIANCE_ACTIVE;
            case BLUE_SHIFT -> LEDModes.BLUE_ALLIANCE_ACTIVE;
            case ENDGAME_SHIFT -> LEDModes.ENDGAME_ACTIVE;
        };
    }
}
