package frc.lib.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private LEDPattern pattern;

    public Led(int port, int length) {
        led = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);
        led.setLength(length);
        led.setData(buffer);
        led.start();
        setColor(Color.kBlack);
    }

    public void setColor(Color c) {
        pattern = LEDPattern.solid(c);
    }

    public void setPattern(LEDPattern p) {
        pattern = p;
    }

    @Override
    public void periodic() {
        pattern.applyTo(buffer);
        led.setData(buffer);
    }
}
