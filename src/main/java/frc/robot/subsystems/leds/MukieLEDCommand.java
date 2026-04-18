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

    private final Color8Bit red;
    private final Color8Bit blue;
    private final Color8Bit purple;

    public MukieLEDCommand(LED leds,  int startIndex, int endIndex, double whiteLevel) {
        this.leds = leds;
        this.startIndex = startIndex;
        this.endIndex = endIndex;
        middleIndex = (startIndex + endIndex) / 2;
        this.whiteLevel = whiteLevel;
        loopCount = 0;

        red = new Color8Bit(255, 0, 0);
        blue = new Color8Bit(0, 0, 255);
        purple = new Color8Bit(255, 0, 255);
    }

    @Override
    public void initialize() {
        leds.setColor(startIndex, startIndex, red);
        leds.setColor(endIndex, endIndex, blue);
    }

    @Override
    public void execute() {
        if (loopCount < (endIndex -  startIndex) / 2 ) {
            // move red and blue
        } else if (loopCount == (endIndex - startIndex) / 2) {
            // create one purple
        } else {
            // randomize purple and white
        }
    }

    private void initialUpdater() {

    }
}

/*
lets assume leds 0-7
8 leds
0 is red
7 is blue
at 0
1 is red and 6 is blue at 1
2 is red and 5 is blue at 2
3 is red and 4 is blue at 3
so at 4 it should be purple
which is 8 / 2
************
assume leds 3-9
7 leds
3 is red
9 is blue
that would be at 0
4 is red and 8 is blue at 1
5 is red and 7 is blue at 2
6 is red and 6 is blue at 3
so 3 should be purple
which is 7 / 2 w int divisiom
 */