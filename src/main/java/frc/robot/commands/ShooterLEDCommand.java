package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDModes;
import java.util.ArrayList;
import java.util.Arrays;

public class ShooterLEDCommand extends Command {

    private final LED leds;
    private final int[] green = {0, 255, 0};
    private final int[] white = {255, 255, 255};
    private ArrayList<int[]> colorQueue;

    public ShooterLEDCommand(LED leds) {
        this.leds = leds;
        addRequirements(leds);
    }

    @Override
    public void initialize() {
        int[][] initialState = new int[leds.getBufferLength()][3];
        int[] currRGB = green.clone();

        int end = (int) Math.ceil(leds.getBufferLength() / 2.0);
        int[] increments = {
                (white[0] - green[0]) / end, (white[1] - green[1]) / end, (white[2] - green[2]) / end
        };

        for (int i = 0; i < end; i++) {
            initialState[i] = new int[] {currRGB[0], currRGB[1], currRGB[2]};

            for (int j = 0; j < 3; j++) {
                currRGB[j] += increments[j];
            }
        }

        int idx = (leds.getBufferLength() / 2) - 1;
        for (int i = end; i < leds.getBufferLength(); i++) {
            initialState[i] =
                    new int[] {initialState[idx][0], initialState[idx][1], initialState[idx][2]};
            idx--;

            colorQueue = new ArrayList<int[]>(Arrays.asList(initialState));
        }
    }

    @Override
    public void execute() {
//        if (ShooterSubsystem.atSetPoint
//                && Math.abs(Limelight.getDistance() - ShooterConstants.SHOOTER_HIGH_DIST)
//                <= ShooterConstants.SHOOTER_DIST_TOLERANCE) {
        if (0 == 0) {
            // wave effect
            for (int i = 0; i < leds.getBufferLength(); i++) {
                leds.setRGB(i, colorQueue.get(i)[0], colorQueue.get(i)[1], colorQueue.get(i)[2]);
            }
            colorQueue.add(0, colorQueue.remove(leds.getBufferLength() - 1));
        } else if (1 == 0) { // Reference 2022 Aurora code for actual if statement bc i accidentally deleted it
            leds.runPattern(LEDModes.LOCKED_GREEN);
        } else {
            leds.runPattern(LEDModes.NOT_LOCKED_RED);
        }
        leds.sendData();
    }
}