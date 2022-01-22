package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;

// LEDs for driver feedback, programmer like a regular Spark Motor controller and PWM output
// Flash lights when ball is picked up, this is to be detected by detecting a CURRENT spike (momentary increase) on the intake motors
// We will have to see what baseline current is, then see what the spike is when we get a ball. Might also have to do some 
// simple filtering where we ignore current spike on start up (1s ignore) because that also draws large current
// Also start flashing yellow or something when 30 seconds left

public class LedSubsystem implements Subsystem  {

    private static final int numberLeds = 40;

    private static final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(numberLeds);
    private static final AddressableLED m_led = new AddressableLED(9); // change to PWM Port for LED Strip

    private static boolean isFlashing = false;

    public LedSubsystem() {
        m_led.setLength(numberLeds);
        m_led.start();
    }

    public void setAllLeds(int red, int green, int blue) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, red, green, blue);
        }
    }

    // A quick 3 flash when ball is picked up.
    public void ballPickUpFlash() {
        if (isFlashing) { return; }
        int count = 3;
        while (count > 0) {
            setAllLeds(0, 255, 0);
            Timer.delay(0.5);
            count++;
        }
    }

    public void startFlashing() {
        while (isFlashing) {
            setAllLeds(255, 255, 0);
            Timer.delay(0.5);
        }
    }

    public void endFlashing() {
        isFlashing = false;
    }

    @Override
    public void periodic() {
        m_led.setData(m_ledBuffer);
    }

}
