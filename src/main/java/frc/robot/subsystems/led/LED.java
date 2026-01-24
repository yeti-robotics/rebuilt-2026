package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ShiftHandler;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {
    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    private final AddressableLEDBufferView leftStrip;
    private final AddressableLEDBufferView rightStrip;
    private LEDModes currentLEDMode;
    private LEDModes temporaryLEDMode;
    private final ShiftHandler shiftHandler;
    private double temporaryPatternEndTime = 0;

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
    }

    @Override
    public void periodic() {
        if (temporaryLEDMode != null && System.currentTimeMillis() >= temporaryPatternEndTime) {
            clearTemporaryPattern();
        }
        if (temporaryLEDMode == null) {
            ShiftHandler.ShiftType currentShift = shiftHandler.getCurrentShift();
            if (currentShift != null) {
                LEDModes computedLEDMode = ShiftHandler.mapShiftToLED(currentShift);
                if (currentLEDMode != computedLEDMode) {
                    applyPattern(computedLEDMode);
                }
            }
        }

        if (currentLEDMode != null && currentLEDMode.isAnimation) {
            updateCurrentPattern();
        }
    }

    private void applyPattern(LEDModes mode) {
        currentLEDMode = mode;
        updateCurrentPattern();
    }

    public void setTemporaryPattern(LEDModes mode, double duration) {
        this.temporaryLEDMode = mode;
        this.temporaryPatternEndTime = System.currentTimeMillis();
        applyPattern(mode);
    }

    public void clearTemporaryPattern() {
        this.temporaryLEDMode = null;
    }

    private void updateCurrentPattern() {
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
