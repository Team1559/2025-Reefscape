package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;

public class Lights {
    private final AddressableLED leds;

    public Lights(int port, int length){
        leds = new AddressableLED(port);
        leds.setLength(length);
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);
        // LEDPattern pattern = LEDPattern.gradient(n, null)
    }

}
