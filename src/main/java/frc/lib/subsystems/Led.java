package frc.lib.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class Led {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    public Led(int port, int length) {
        led = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);
        led.setLength(length);
        led.setData(buffer);
        led.start();
    }
    public void setColor(Color c){
        LEDPattern pattern = LEDPattern.solid(c);
        pattern.applyTo(buffer);
        led.setData(buffer);
    }
    
}
