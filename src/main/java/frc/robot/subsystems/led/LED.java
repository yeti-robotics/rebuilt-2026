package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ShiftHandler;

public class LED extends SubsystemBase {
    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    private final AddressableLEDBufferView leftStrip;
    private final AddressableLEDBufferView rightStrip;
    private LEDModes currentLEDMode;
    private final ShiftHandler shiftHandler;
    private LEDModes previousLEDMode;
    private ShiftHandler.ShiftType previousShift;
    private double animationStartTime;
    private boolean isTimedAnimation;

    public LED() {
        ledStrip = new AddressableLED(LEDConstants.LED_STRIP_PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);
        leftStrip = ledBuffer.createView(0, (LEDConstants.LED_COUNT / 2) - 1);
        rightStrip = ledBuffer
                .createView(LEDConstants.LED_COUNT / 2, LEDConstants.LED_COUNT - 1)
                .reversed();

        shiftHandler = new ShiftHandler();
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.start();
        setDefaultCommand(allianceShifts());
    }

    private Command allianceShifts() {
        return run(() -> {
            if (isTimedAnimation && currentLEDMode == null) {
                double currentTime = Timer.getFPGATimestamp();
                if (currentTime - animationStartTime >= LEDConstants.ANIMATION_TIMEOUT_SECONDS) {
                    isTimedAnimation = false;
                    return;
                }
            }

            ShiftHandler.ShiftType currentShift = shiftHandler.getCurrentShift();
            if (currentShift != null && currentShift != ShiftHandler.ShiftType.UNKNOWN) {
                if (shouldShowDeactivationWarning()) {
                    if (currentLEDMode != LEDModes.ALLIANCE_DEACTIVATION_WARNING) {
                        applyPattern(LEDModes.ALLIANCE_DEACTIVATION_WARNING);
                    }
                } else {
                    LEDModes computedLEDMode = mapShiftToLED(currentShift);
                    if (currentLEDMode != computedLEDMode) {
                        applyPattern(computedLEDMode);
                    }
                }
                if (previousShift != currentShift) {
                    previousShift = currentShift;
                }
            }

            if (currentLEDMode != null && currentLEDMode.isAnimation) {
                updateCurrentPattern();
            }
        });
    }

    private LEDModes mapShiftToLED(ShiftHandler.ShiftType shift) {
        if (shift == null) {
            return LEDModes.BLINKING_ORANGE;
        }
        return switch (shift) {
            case TRANSITION_SHIFT -> LEDModes.TRANSITION_ACTIVE;
            case RED_SHIFT -> LEDModes.RED_ALLIANCE_ACTIVE;
            case BLUE_SHIFT -> LEDModes.BLUE_ALLIANCE_ACTIVE;
            case ENDGAME_SHIFT -> LEDModes.ENDGAME_ACTIVE;
            case UNKNOWN -> LEDModes.BLINKING_ORANGE;
        };
    }

    private boolean shouldShowDeactivationWarning() {
        if (DriverStation.isTeleopEnabled()) {
            double matchTime = DriverStation.getMatchTime();

            if (matchTime > 30 && matchTime < 130) {
                int blockTime = (int) (130 - matchTime) / 25;
                double timeInBlock = 130 - matchTime - (blockTime * 25);

                return timeInBlock >= 22;
            }
        }

        return false;
    }

    private void applyPattern(LEDModes mode) {
        currentLEDMode = mode;
        updateCurrentPattern();
    }

    private void updateCurrentPattern() {
        if (currentLEDMode == null) return;

        if (isTimedAnimation && currentLEDMode.isAnimation) {
            double currentTime = Timer.getFPGATimestamp();
            if (currentTime - animationStartTime >= LEDConstants.ANIMATION_TIMEOUT_SECONDS) {
                isTimedAnimation = false;
                if (previousLEDMode != null && !previousLEDMode.isAnimation) {
                    applyPattern(previousLEDMode);
                } else {
                    currentLEDMode = null;
                }
                return;
            }
        }

        currentLEDMode.pattern.applyTo(leftStrip);
        currentLEDMode.pattern.applyTo(rightStrip);
        sendData();
    }

    public void sendData() {
        ledStrip.setData(ledBuffer);
    }

    public Command runPattern(LEDModes pattern) {
        return run(() -> {
                    if (!isTimedAnimation || currentLEDMode != pattern) {
                        previousLEDMode = currentLEDMode;
                        animationStartTime = Timer.getFPGATimestamp();
                        isTimedAnimation = true;
                        currentLEDMode = pattern;
                    }

                    updateCurrentPattern();
                })
                .ignoringDisable(true);
    }
}
