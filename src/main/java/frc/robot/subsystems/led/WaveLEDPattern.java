package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import java.util.ArrayList;
import java.util.Arrays;

public class WaveLEDPattern implements LEDPattern {
    private ArrayList<int[]> colorQueue;
    private boolean isConfigured = false;

    private void configureWaveLED(int bufferLength) {
        final int[] green = {0, 255, 0};
        final int[] white = {255, 255, 255};
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
        for (int i = end; i < bufferLength; i++) {
            initialState[i] = new int[] {initialState[idx][0], initialState[idx][1], initialState[idx][2]};
            idx--;
        }

        colorQueue = new ArrayList<>(Arrays.asList(initialState));
    }

    @Override
    public void applyTo(LEDReader reader, LEDWriter writer) {
        if (!isConfigured) {
            configureWaveLED(reader.getLength());
            isConfigured = true;
        }
        int bufferLength = reader.getLength();
        for (int i = 0; i < reader.getLength(); i++) {
            writer.setRGB(
                    i, colorQueue.get(i)[0], colorQueue.get(i)[1], colorQueue.get(i)[2]);
        }
        colorQueue.add(0, colorQueue.remove(bufferLength - 1));
    }
}
