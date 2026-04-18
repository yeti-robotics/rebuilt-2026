package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;

public class MukieLEDCommand extends Command {
    private final LED leds;
    private final int startIndex;
    private final int endIndex;
    private final int middleIndex;
    private final double whiteLevel;
    private int loopCount;
    private int iterations;

    private final Color8Bit red = new Color8Bit(255, 0, 0);
    private final Color8Bit blue = new Color8Bit(0, 0, 255);
    private final Color8Bit purple= new Color8Bit(106, 6, 124);
    private final Color8Bit empty = new Color8Bit(0, 0, 0);

    public MukieLEDCommand(LED leds,  int startIndex, int endIndex, double whiteLevel) {
        this.leds = leds;
        this.startIndex = startIndex;
        this.endIndex = endIndex;
        middleIndex = (startIndex + endIndex) / 2;
        this.whiteLevel = whiteLevel;
        loopCount = 0;
        iterations = 0;
    }

    @Override
    public void initialize() {
        leds.setColor(startIndex, startIndex, red, 1);
        leds.setColor(endIndex, endIndex, blue, 1);
    }

    @Override
    public void execute() {
        iterations++;
        if (iterations % 25 == 0) {
            if (loopCount++ < (endIndex - startIndex) / 2) {
                moveToCenter();
            } else if (loopCount++ == (endIndex - startIndex) / 2) {
                leds.setColor(startIndex, endIndex, empty, 0);
                leds.setColor(middleIndex, middleIndex, purple, 1);
            } else {
                iterations++;
                purple();
            }
        }
    }

    private void moveToCenter() {
        leds.setColor(startIndex, startIndex, empty, 0);
        leds.setColor(startIndex + loopCount, startIndex + loopCount, red, 1);
        leds.setColor(endIndex, endIndex, empty, 0);
        leds.setColor(endIndex - loopCount, endIndex - loopCount, blue, 1);
    }

    private void purple() {
        for (int i = startIndex; i <=  endIndex; i++) {
            leds.setColor(i, i, purple, Math.max(0.7, Math.random()));
        }
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
