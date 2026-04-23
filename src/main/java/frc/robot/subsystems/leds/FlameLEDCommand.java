package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;

public class FlameLEDCommand extends Command {
    private final LED leds;
    private final int ledStartIndex;
    private final int ledEndIndex;
    private final double sparking;
    private final double cooling;
    private final double brightness;

    // Color customization
    private final Color8Bit baseColor;
    private final Color8Bit sparkColor;
    private final Color8Bit emberColor;
    private final double colorMix;

    // Flame state
    private double[] flameBuffer;
    private int frameCount;

    public FlameLEDCommand(
            LED leds,
            int ledStartIndex,
            int ledEndIndex,
            double sparking,
            double cooling,
            double brightness,
            Color8Bit baseColor,
            Color8Bit sparkColor,
            Color8Bit emberColor,
            double colorMix) {
        this.leds = leds;
        this.ledStartIndex = ledStartIndex;
        this.ledEndIndex = ledEndIndex;
        this.sparking = sparking;
        this.cooling = cooling;
        this.brightness = brightness;
        this.baseColor = baseColor;
        this.sparkColor = sparkColor;
        this.emberColor = emberColor;
        this.colorMix = colorMix;

        this.flameBuffer = new double[ledEndIndex - ledStartIndex + 1];
        this.frameCount = 0;

        addRequirements(leds);
    }

    // Simplified constructor with default colors (red/orange fire)
    public FlameLEDCommand(
            LED leds, int ledStartIndex, int ledEndIndex, double sparking, double cooling, double brightness) {
        this(
                leds,
                ledStartIndex,
                ledEndIndex,
                sparking,
                cooling,
                brightness,
                new Color8Bit(255, 100, 0), // Orange base
                new Color8Bit(255, 255, 200), // Yellow-white sparks
                new Color8Bit(100, 0, 0), // Dark red embers
                0.5);
    }

    @Override
    public void initialize() {
        frameCount = 0;
        for (int i = 0; i < flameBuffer.length; i++) {
            flameBuffer[i] = 0;
        }
    }

    @Override
    public void execute() {
        updateFlameBuffer();
        renderFlame();
        frameCount++;
    }

    private void updateFlameBuffer() {
        // Cool existing flames
        for (int i = 0; i < flameBuffer.length; i++) {
            flameBuffer[i] = Math.max(0, flameBuffer[i] - cooling);
        }

        // Add new sparks at the bottom
        for (int i = 0; i < 3; i++) {
            if (Math.random() < sparking) {
                int sparkIndex = (int) (Math.random() * Math.min(5, flameBuffer.length));
                flameBuffer[sparkIndex] = 1.0;
            }
        }

        // Propagate flames upward
        for (int i = flameBuffer.length - 1; i > 0; i--) {
            if (flameBuffer[i - 1] > 0) {
                flameBuffer[i] = Math.max(flameBuffer[i], flameBuffer[i - 1] * 0.7);
            }
        }
    }

    private void renderFlame() {
        for (int i = 0; i < flameBuffer.length; i++) {
            double intensity = flameBuffer[i];
            if (intensity > 0.01) {
                // Mix colors based on intensity
                Color8Bit color = mixColors(intensity);
                int r = (int) (color.red * brightness);
                int g = (int) (color.green * brightness);
                int b = (int) (color.blue * brightness);
                leds.setLEDs(ledStartIndex + i, 1, r, g, b);
            }
        }
    }

    private Color8Bit mixColors(double intensity) {
        // Mix between ember color, base color, and spark color based on intensity
        if (intensity < 0.3) {
            return emberColor;
        } else if (intensity < 0.7) {
            return lerpColor(emberColor, baseColor, (intensity - 0.3) / 0.4);
        } else {
            return lerpColor(baseColor, sparkColor, (intensity - 0.7) / 0.3);
        }
    }

    private Color8Bit lerpColor(Color8Bit c1, Color8Bit c2, double t) {
        int r = (int) (c1.red + (c2.red - c1.red) * t);
        int g = (int) (c1.green + (c2.green - c1.green) * t);
        int b = (int) (c1.blue + (c2.blue - c1.blue) * t);
        return new Color8Bit(
                Math.max(0, Math.min(255, r)), Math.max(0, Math.min(255, g)), Math.max(0, Math.min(255, b)));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        leds.clearLEDs();
    }
}
