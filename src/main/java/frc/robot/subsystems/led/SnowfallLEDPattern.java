package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;

public class SnowfallLEDPattern implements LEDPattern {
    long waitTime;
    long startTime;
    int stage;
    boolean isConfigured = false;

    public SnowfallLEDPattern() {
        waitTime = 2;
        startTime = System.currentTimeMillis();
        stage = 0;
    }

    private void configureSnowfall(LEDReader reader, LEDWriter writer) {
        for (int i = 0; i < reader.getLength(); i++) {
            if (i % 4 == stage) {
                writer.setRGB(i, 255, 255, 255);
                continue;
            }
            writer.setRGB(i, 20, 120, 255);
        }
        stage++;
    }

    @Override
    public void applyTo(LEDReader reader, LEDWriter writer) {
        if (!isConfigured) {
            configureSnowfall(reader, writer);
            isConfigured = true;
        }
        if (System.currentTimeMillis() - startTime >= waitTime) {
            for (int i = 0; i < reader.getLength(); i++) {
                if (i % 4 == stage) {
                    writer.setRGB(i, 255, 255, 255);
                    continue;
                }
                writer.setRGB(i, 20, 120, 255);
            }
            stage = stage + 1 > 3 ? 0 : stage + 1;
            startTime = System.currentTimeMillis();
        }
    }
}
