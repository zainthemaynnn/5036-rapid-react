package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
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

    // A quick 3 flash when ball is picked up.
    public void ballPickUpFlash() {
        if (isFlashing) { return; }
        int count = 3;
        while (count > 0) {
            m_led.set(ScoreColourSpeed);
            Timer.delay(0.5);
            m_led.set(0);
            Timer.delay(0.5);
            count++;
        }
    }

    public void startFlashing() {
        while (isFlashing) {
            m_led.set(TimeColourSpeed);
            Timer.delay(0.5);
            m_led.set(0);
            Timer.delay(0.5);
        }
    }

    public void endFlashing() {
        isFlashing = false;
    }

    @Override
    public void periodic() {    

    }

}
