package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Subsystem;

// LEDs for driver feedback, programmer like a regular Spark Motor controller and PWM output
// Flash lights when ball is picked up, this is to be detected by detecting a CURRENT spike (momentary increase) on the intake motors
// We will have to see what baseline current is, then see what the spike is when we get a ball. Might also have to do some 
// simple filtering where we ignore current spike on start up (1s ignore) because that also draws large current
// Also start flashing yellow or something when 30 seconds left

public class LedSubsystem implements Subsystem  {

    private static final Spark m_led = new Spark(56);
    private static boolean isFlashing = false;
    
    public static final double bootUpColourSpeed = 0.5;
    public static final double ScoreColourSpeed = 0.4;
    public static final double TimeColourSpeed = 0.3;

    // https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf 
    // for all the LED Information

    public LedSubsystem() { 

    }

    public void flashTimes(double colour, int times, double duration) {
        if (isFlashing) { return; }
        for (int i = 0; i < times; i++) {
            m_led.set(colour);
            Timer.delay(duration);
            m_led.set(0);
            Timer.delay(duration);
        }
    }

    // A quick 3 flash when ball is picked up.
    public void ballPickUpFlash() {
        flashTimes(ScoreColourSpeed, 3, 0.5);
    }

    public void startFlashing() {
        if (isFlashing) { return; }
        while (isFlashing) {
            flashTimes(ScoreColourSpeed, 1, 0.5);
        }
    }

    public void bootUpFlash() {
        flashTimes(bootUpColourSpeed, 3, 0.5);
    }

    public void endFlashing() {
        isFlashing = false;
    }

    @Override
    public void periodic() {    

    }
}
