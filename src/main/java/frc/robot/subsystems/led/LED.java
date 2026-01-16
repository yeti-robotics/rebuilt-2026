package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.*;
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
        ShiftHandler.ShiftType currentShift = shiftHandler.getCurrentShift();

        if (shiftHandler.getCurrentShift() != null) {
            LEDModes computedLEDMode = ShiftHandler.mapShiftToLED(currentShift);
            if (currentLEDMode != computedLEDMode) {
                applyPattern(computedLEDMode);
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
