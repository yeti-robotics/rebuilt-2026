package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    public AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    private final AddressableLEDBufferView leftStrip;
    private final AddressableLEDBufferView rightStrip;
    private LEDModes currentLEDMode;

    public LED() {
        ledStrip = new AddressableLED(LEDConstants.LED_STRIP_PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);
        leftStrip = ledBuffer.createView(0, (LEDConstants.LED_COUNT / 2) - 1);
        rightStrip = ledBuffer.createView(LEDConstants.LED_COUNT / 2, LEDConstants.LED_COUNT - 1).reversed();

        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.start();
    }


    // Maybe get rid of this enum and use LEDModes instead
    public enum ShiftType {
        TRANSITION_SHIFT,
        RED_SHIFT,
        BLUE_SHIFT,
        ENDGAME_SHIFT;

        public ShiftType invert() {
            return this == RED_SHIFT ? BLUE_SHIFT : RED_SHIFT;
        }
    }

    private ShiftType firstShift = ShiftType.TRANSITION_SHIFT;
    boolean teleopDataRead = false;


    @Override
    public void periodic() {
        if (currentLEDMode != null && currentLEDMode.isAnimation) {
            updateCurrentPattern();
        }

        ShiftType currentShift = null;

        if (DriverStation.isTeleopEnabled()) {
           double matchTime =  DriverStation.getMatchTime();

           if (!teleopDataRead) {
               String gameData = DriverStation.getGameSpecificMessage();

               if (!gameData.isEmpty()) {
                   switch (gameData.charAt(0)) {
                       case 'B':
                           firstShift = ShiftType.RED_SHIFT;
                           teleopDataRead = true;
                           break;
                       case 'R':
                           firstShift = ShiftType.BLUE_SHIFT;
                           teleopDataRead = true;
                           break;
                       default:
                           break;
                   }
               }
           }

            //           140 - 10 = 130 transition
            //
            //           130 - 25 = 105 first 130 - 105
            //           105 - 25 = 80 second 105 - 80
            //           80 - 25  = 55 third  80 - 55
            //           55 - 25 =  30 fourth 55 - 30

            //           and then end game for 30 seconds end game

           if (matchTime >= 130) {
               currentShift = ShiftType.TRANSITION_SHIFT;
           } else if (matchTime <= 30) {
               currentShift = ShiftType.ENDGAME_SHIFT;
           } else {
               int blockTime = (int) (130.0 - matchTime) / 25; // split into 25 second blocks
               currentShift = blockTime % 2 == 0 ? firstShift : firstShift.invert();
           }
        }

        if (currentLEDMode != null && currentLEDMode.isAnimation) {
            updateCurrentPattern();
        }

        if (currentShift != null) {
            LEDModes computedLEDMode = mapShiftToLED(currentShift);

            if (currentLEDMode != computedLEDMode) {
                applyPattern(computedLEDMode);
            }
        }
    }

    public LEDModes mapShiftToLED(ShiftType shiftType) {
        return switch (shiftType) {
            case TRANSITION_SHIFT -> LEDModes.TRANSITION_ACTIVE;
            case RED_SHIFT -> LEDModes.RED_ALLIANCE_ACTIVE;
            case BLUE_SHIFT -> LEDModes.BLUE_ALLIANCE_ACTIVE;
            case ENDGAME_SHIFT -> LEDModes.ENDGAME_ACTIVE;
        };
    }

    public void applyPattern(LEDModes mode) {
        currentLEDMode = mode;

        updateCurrentPattern();
    }

    public void updateCurrentPattern() {
        if (currentLEDMode == null) return;

        currentLEDMode.pattern.applyTo(leftStrip);
        currentLEDMode.pattern.applyTo(rightStrip);

        ledStrip.setData(ledBuffer);
    }

    public Command runPattern(LEDModes pattern) {
        return runOnce(() -> applyPattern(pattern)).ignoringDisable(true);
    }
}
