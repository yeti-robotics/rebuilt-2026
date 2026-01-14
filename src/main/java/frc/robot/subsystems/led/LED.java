package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    public AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    private final AddressableLEDBufferView leftStrip;
    private final AddressableLEDBufferView rightStrip;

    public LED() {
        ledStrip = new AddressableLED(LEDConstants.LED_STRIP_PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);
        leftStrip = ledBuffer.createView(0, (LEDConstants.LED_COUNT / 2) - 1);
        rightStrip = ledBuffer.createView(LEDConstants.LED_COUNT / 2, LEDConstants.LED_COUNT - 1).reversed();
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    @Override
    public void periodic() {
        ledStrip.setData(ledBuffer);

        String gameData;
        gameData = DriverStation.getGameSpecificMessage();
        if (!gameData.isEmpty()) {
            switch (gameData.charAt(0)) {
                case 'B':
                    applyPattern(LEDModes.BLUE_ALLIANCE_ACTIVE.pattern);
                case 'R':
                    applyPattern(LEDModes.RED_ALLIANCE_ACTIVE.pattern);
                default:
                    break;
            }
        }
    }

    public void applyPattern(LEDPattern pattern) {
        pattern.applyTo(leftStrip);
        pattern.applyTo(rightStrip);
    }

    public Command runPattern(LEDModes pattern) {
        return runOnce(() -> applyPattern(pattern.pattern)).repeatedly().ignoringDisable(true);
    }
}
