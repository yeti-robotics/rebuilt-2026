package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    private final AddressableLEDBufferView leftStrip;
    private final AddressableLEDBufferView rightStrip;
    private LEDModes currentLEDMode;

    public LED() {
        ledStrip = new AddressableLED(LEDConstants.LED_STRIP_PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);
        leftStrip = ledBuffer.createView(0, (LEDConstants.LED_COUNT / 2) - 1);
        rightStrip = ledBuffer
                .createView(LEDConstants.LED_COUNT / 2, LEDConstants.LED_COUNT - 1)
                .reversed();
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.start();
    }

    public enum ShiftType {
        TRANSITION_SHIFT,
        RED_SHIFT,
        BLUE_SHIFT,
        ENDGAME_SHIFT;

        public ShiftType invert() {
            return this == RED_SHIFT ? BLUE_SHIFT : RED_SHIFT;
        }
    }

    private ShiftType firstShift = ShiftType.TRANSITION_SHIFT;
    boolean teleopDataRead = false;

    @Override
    public void periodic() {
        if (currentLEDMode != null && currentLEDMode.isAnimation) {
            updateCurrentPattern();
        }
        ShiftType currentShift = null;

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

        if (currentLEDMode != null && currentLEDMode.isAnimation) {
            updateCurrentPattern();
        }

        if (currentShift != null) {
            LEDModes computedLEDMode = mapShiftToLED(currentShift);
            if (currentLEDMode != computedLEDMode) {
                applyPattern(computedLEDMode);
            }
        }
    }

    public LEDModes mapShiftToLED(ShiftType shift) {
        return switch (shift) {
            case TRANSITION_SHIFT -> LEDModes.TRANSITION_ACTIVE;
            case RED_SHIFT -> LEDModes.RED_ALLIANCE_ACTIVE;
            case BLUE_SHIFT -> LEDModes.BLUE_ALLIANCE_ACTIVE;
            case ENDGAME_SHIFT -> LEDModes.ENDGAME_ACTIVE;
        };
    }

    private void applyPattern(LEDModes mode) {
        currentLEDMode = mode;
        updateCurrentPattern();
    }

    public void updateCurrentPattern() {
        if (currentLEDMode == null) return;

        currentLEDMode.pattern.applyTo(leftStrip);
        currentLEDMode.pattern.applyTo(rightStrip);

        sendData();
    }

    public int getBufferLength() {
        return ledBuffer.getLength();
    }

    public void setRGB(int i, int r, int g, int b) {
        ledBuffer.setRGB(i, r, g, b);
    }

    public void sendData() {
        ledStrip.setData(ledBuffer);
    }

    public Command runPattern(LEDModes pattern) {
        return runOnce(() -> applyPattern(pattern)).ignoringDisable(true);
    }
}
