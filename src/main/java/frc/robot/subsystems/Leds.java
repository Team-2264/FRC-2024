package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for interacting with Addressable LEDs
 */
public class Leds extends SubsystemBase {
    private  AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;
    private float scaleFactor = 1.0f;
    private int currentRed = 0;
    private int currentGreen = 0;
    private int currentBlue = 0;

    private int rainbowFirstPixelHue = 0;

    public Leds(int pwmPort, int numLeds, float scaleFactor) {

        ledBuffer = new AddressableLEDBuffer(numLeds);
        ledStrip = new AddressableLED(pwmPort);

        // Length is expensive to set, so only set it once, then just update data
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    public boolean setRGB(int red, int green, int blue)
    {
        int r = Math.round(red   * scaleFactor);
        int g = Math.round(green * scaleFactor);
        int b = Math.round(blue  * scaleFactor);

        // check if we are trying to set the rgb the same as it is already set.
        if ((r == currentRed) && (g == currentGreen) && (b == currentBlue))
        {
            return true;
        }

        System.out.println("SetRGB: r=" + red + "g=" + green + "b=" + blue);

        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values as specified
            ledBuffer.setRGB(i, r, g, b);
         }

         currentRed = r;
         currentGreen = g;
         currentBlue = b;
         
         ledStrip.setData(ledBuffer);

         return true;
    }

    public void Rainbow() {
        // For every pixel
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess

            final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;  
                // Set the value
                ledBuffer.setHSV(i, hue, 255, 128);
    
        }
    
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
    
        // Check bounds
        rainbowFirstPixelHue %= 180;

        ledStrip.setData(ledBuffer);
    }
    
}
