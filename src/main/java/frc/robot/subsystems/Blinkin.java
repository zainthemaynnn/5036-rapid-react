package frc.robot.subsystems;

import java.util.PriorityQueue;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Subsystem;

// LEDs for driver feedback, programmer like a regular Spark Motor controller and PWM output
// Flash lights when ball is picked up, this is to be detected by detecting a CURRENT spike (momentary increase) on the intake motors
// We will have to see what baseline current is, then see what the spike is when we get a ball. Might also have to do some 
// simple filtering where we ignore current spike on start up (1s ignore) because that also draws large current
// Also start flashing yellow or something when 30 seconds left

public class Blinkin implements Subsystem  {
    private Spark blinkin;

    public static enum BlinkinColor {
        DEFAULT (-.87),
        OFF     (0),
        BOOT    (.5),
        SCORE   (.4),
        ENDGAME (.3);

        private double sparkValue;

        private BlinkinColor(double sparkValue) {
            this.sparkValue = sparkValue;
        }

        public double sparkValue() {
            return sparkValue;
        }
    }

    private PriorityQueue<BlinkinColor> queue;

    // https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf 
    // for all the LED Information

    public Blinkin(int port) { 
        blinkin = new Spark(port);
        queue = new PriorityQueue<BlinkinColor>(BlinkinColor::compareTo);
        addColor(BlinkinColor.DEFAULT);
    }

    public void addColor(BlinkinColor color) {
        queue.add(color);
    }

    public void removeColor(BlinkinColor color) {
        queue.remove(color);
    }

    @Override
    public void periodic() {
        blinkin.set(queue.peek().sparkValue());
    }
}
