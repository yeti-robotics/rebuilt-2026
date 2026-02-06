package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;

public class SnowfallLEDPattern implements LEDPattern {
    long waitTime;
    long startTime;
    int stage;

    public SnowfallLEDPattern() {
        waitTime = 50;
        startTime = System.currentTimeMillis();
        stage = 0;
    }

    @Override
    public void applyTo(LEDReader reader, LEDWriter writer) {
        for (int i = 0; i < reader.getLength(); i++) {
            if (i % 4 == stage) {
                writer.setRGB(i, 255, 255, 255);
                continue;
            }
            writer.setRGB(i, 20, 120, 255);
        }
        if (System.currentTimeMillis() - startTime >= waitTime) {
            stage = (stage + 1) % 4;
            startTime = System.currentTimeMillis();
        }
    }
}
