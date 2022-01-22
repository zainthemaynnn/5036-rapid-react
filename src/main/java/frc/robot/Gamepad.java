package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Gamepad {
    private Joystick joystick;

    public static enum Axis {
        LEFT_X,
        LEFT_Y,
        L2,
        R2,
        RIGHT_X,
        RIGHT_Y,
    }

    public static enum Button {
        _z, // what button does this even go to?
        GREEN,
        RED,
        BLUE,
        YELLOW,
        L1,
        L2,
        BACK,
        START,
    }

    public static enum POV {
        UP,
        UP_RIGHT,
        RIGHT,
        DOWN_RIGHT,
        DOWN,
        DOWN_LEFT,
        LEFT,
        UP_LEFT;

        private int heading;

        private POV() {
            heading = ordinal() * 45;
        }

        public int heading() {
            return heading;
        }
    }

    public Gamepad(int port) {
        joystick = new Joystick(port);
    }

    public double getAxis(Axis axis) {
        switch (axis) {
            // invert Y axes
            case LEFT_Y:
            case RIGHT_Y:
                return -joystick.getRawAxis(axis.ordinal());
            default:
                return joystick.getRawAxis(axis.ordinal());
        }
    }

    public double getAxisWithDeadband(Axis axis, double deadband) {
        double v = getAxis(axis);
        return Math.abs(v) > deadband ? v : 0;
    }

    public boolean getButtonPressed(Button button) {
        return joystick.getRawButton(button.ordinal());
    }

    public boolean getPOVPressed(POV direction) {
        return joystick.getPOV(0) == direction.heading();
    }
}
