package frc.robot.subsystems.led;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.ShiftHandler;
import java.util.ArrayList;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {
    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    private final AddressableLEDBufferView leftStrip;
    private final AddressableLEDBufferView rightStrip;
    private LEDModes currentLEDMode;
    private final ShiftHandler shiftHandler;

    double tolerance = 6.0; // TODO: Calculate
    double idealDistanceToHub = 82; // TODO: Calculate

    public LED(ShooterSubsystem shooter, Vision vision) {
        ledStrip = new AddressableLED(LEDConstants.LED_STRIP_PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);
        leftStrip = ledBuffer.createView(0, (LEDConstants.LED_COUNT / 2) - 1);
        rightStrip = ledBuffer
                .createView(LEDConstants.LED_COUNT / 2, LEDConstants.LED_COUNT - 1)
                .reversed();

        shiftHandler = new ShiftHandler();
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.start();
        configureWaveLED();
        setDefaultCommand(runOnce(this::defaultCommand));

        new Trigger(() -> shooter.getVelocity().isNear(Units.RotationsPerSecond.of(120), 0.1))
                .whileTrue(runOnce(() -> applyPattern(LEDModes.LOCKED_GREEN)));

        new Trigger(() -> shooter.getVelocity().isNear(Units.RotationsPerSecond.of(120), 0.1))
                .and(() -> Math.abs(vision.getDistance() - idealDistanceToHub) <= tolerance)
                .whileTrue(runEnd(this::wave, this::defaultCommand));
    }

    private final int[] green = {0, 255, 0};
    private final int[] white = {255, 255, 255};
    private ArrayList<int[]> colorQueue;

    private void configureWaveLED() {
        int bufferLength = getBufferLength();
        int[][] initialState = new int[bufferLength][3];
        int[] currRGB = green.clone();

        int end = (int) Math.ceil(bufferLength / 2.0);
        int[] increments = {(white[0] - green[0]) / end, (white[1] - green[1]) / end, (white[2] - green[2]) / end};

        for (int i = 0; i < end; i++) {
            initialState[i] = new int[] {currRGB[0], currRGB[1], currRGB[2]};

            for (int j = 0; j < 3; j++) {
                currRGB[j] += increments[j];
            }
        }

        int idx = (bufferLength / 2) - 1;
        for (int i = end; i < getBufferLength(); i++) {
            initialState[i] = new int[] {initialState[idx][0], initialState[idx][1], initialState[idx][2]};
            idx--;

            colorQueue = new ArrayList<>(Arrays.asList(initialState));
        }
    }

    private void wave() {
        for (int i = 0; i < getBufferLength(); i++) {
            setRGB(i, colorQueue.get(i)[0], colorQueue.get(i)[1], colorQueue.get(i)[2]);
        }
        colorQueue.add(0, colorQueue.remove(getBufferLength() - 1));
        sendData();
    }

    private void defaultCommand() {
        ShiftHandler.ShiftType currentShift = shiftHandler.getCurrentShift();
        if (shiftHandler.getCurrentShift() != null) {
            LEDModes computedLEDMode = ShiftHandler.mapShiftToLED(currentShift);
            if (currentLEDMode != computedLEDMode) {
                applyPattern(computedLEDMode);
            }
            Logger.recordOutput("Current Shift", currentShift.getShiftString());
        }
        if (currentLEDMode != null && currentLEDMode.isAnimation) {
            updateCurrentPattern();
        }
    }

    private void applyPattern(LEDModes mode) {
        currentLEDMode = mode;
        updateCurrentPattern();
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
