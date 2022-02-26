package frc.robot.subsystems;

import java.util.PriorityQueue;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

// LEDs for driver feedback, programmer like a regular Spark Motor controller and PWM output
// Flash lights when ball is picked up, this is to be detected by detecting a CURRENT spike (momentary increase) on the intake motors
// We will have to see what baseline current is, then see what the spike is when we get a ball. Might also have to do some 
// simple filtering where we ignore current spike on start up (1s ignore) because that also draws large current
// Also start flashing yellow or something when 30 seconds left

public class Blinkin implements Subsystem  {
    private Spark blinkin;

    // TODO: greatness will appear here soon
    private static enum BlinkinMap {
        ;

        private double value;

        private BlinkinMap(double value) {
            this.value = value;
        }

        public double value() {
            return value;
        }
    }

    public static enum BlinkinColor {
        DEFAULT (.99),
        OFF     (.99),
        BOOT    (.35),
        INTAKE  (.77),
        SHOOT   (.35);

        private double value;

        private BlinkinColor(double value) {
            this.value = value;
        }

        public double value() {
            return value;
        }
    }

    private PriorityQueue<BlinkinColor> queue;

    // https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf 
    // for all the LED Information

    public Blinkin(int port) { 
        blinkin = new Spark(port);
        queue = new PriorityQueue<>(BlinkinColor::compareTo);
    }

    public void addColor(BlinkinColor color) {
        queue.add(color);
    }

    public void removeColor(BlinkinColor color) {
        queue.remove(color);
    }

    public boolean containsColor(BlinkinColor color) {
        return queue.contains(color);
    }

    public void display() {
        if (!queue.isEmpty()) {
            blinkin.set(queue.peek().value());
        } else {
            blinkin.set(BlinkinColor.DEFAULT.value());
        }

        SmartDashboard.putNumber("Size", queue.size());
        SmartDashboard.putNumber("Blinkin", blinkin.get());
    }
}
