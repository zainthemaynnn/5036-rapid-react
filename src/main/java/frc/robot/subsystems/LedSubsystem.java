package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Subsystem;

// LEDs for driver feedback, programmer like a regular Spark Motor controller and PWM output
// Flash lights when ball is picked up, this is to be detected by detecting a CURRENT spike (momentary increase) on the intake motors
// We will have to see what baseline current is, then see what the spike is when we get a ball. Might also have to do some 
// simple filtering where we ignore current spike on start up (1s ignore) because that also draws large current
// Also start flashing yellow or something when 30 seconds left

public class LedSubsystem implements Subsystem  {

    private MotorController blinkin;
    private boolean isFlashing = false;

    public static enum BlinkinColor {
        OFF     (0),
        BOOT    (.5),
        SCORE   (.4),
        ENDGAME (.3);

        private double value;

        private BlinkinColor(double value) {
            this.value = value;
        }

        public double value() {
            return value;
        }
    }

    // https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf 
    // for all the LED Information

    public LedSubsystem(MotorController blinkin) { 
        this.blinkin = blinkin;
    }

    public void flashTimes(BlinkinColor color, int times, double duration) {
        if (isFlashing) { return; }
        for (int i = 0; i < times; i++) {
            blinkin.set(color.value());
            Timer.delay(duration);
            blinkin.set(BlinkinColor.OFF.value());
            Timer.delay(duration);
        }
    }

    // A quick 3 flash when ball is picked up.
    public void ballPickUpFlash() {
        flashTimes(BlinkinColor.SCORE, 3, 0.5);
    }

    public void startFlashing(BlinkinColor color) {
        if (isFlashing) { return; }
        while (isFlashing) {
            flashTimes(color, 1, 0.5);
        }
    }

    public void bootUpFlash() {
        flashTimes(BlinkinColor.BOOT, 3, 0.5);
    }

    public void endFlashing() {
        isFlashing = false;
    }
}
