package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Limelight implements Subsystem {
    private class Target {
        public double x, y;
        public double area;
        public boolean exists;

        public Target(double x, double y, double area, boolean exists) {
            this.x = x;
            this.y = y;
            this.area = area;
            this.exists = exists;
        }
    }

    public enum LEDState {
        DEFAULT,
        OFF,
        ON,
        BLINK;
    }

    public enum CameraMode {
        VISION,
        DRIVER;
    }

    private final static String NETWORK_TABLE_NAME = "limelight";

    private final NetworkTable feed;
    public Target target;

    public Limelight() {
        feed = NetworkTableInstance.getDefault().getTable(NETWORK_TABLE_NAME);
    }

    public void setLEDState(LEDState ledState) {
        feed.getEntry("ledMode").setNumber(ledState.ordinal());
    }

    public void setCameraMode(CameraMode camMode) {
        feed.getEntry("cameraMode").setNumber(camMode.ordinal());
    }

    public void periodic() {
        target = new Target(
            feed.getEntry("tx").getDouble(0.0),
            feed.getEntry("ty").getDouble(0.0),
            feed.getEntry("ta").getDouble(0.0),
            feed.getEntry("tv").getBoolean(false)
        );
    }
}
