package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Controller {
    private Joystick joystick;

    public static enum Axis {
        LeftX,
        LeftY,
        L2,
        R2,
        RightX,
        RightY,
    }

    public static enum Button {
        _z, // what button does this even go to?
        Green,
        Red,
        Blue,
        Yellow,
        L1,
        L2,
        Back,
        Start,
    }

    public static enum POV {
        Up,
        UpRight,
        Right,
        DownRight,
        Down,
        DownLeft,
        Left,
        UpLeft;

        private int heading;

        private POV() {
            heading = ordinal() * 45;
        }

        public int heading() {
            return heading;
        }
    }

    public Controller(int port) {
        joystick = new Joystick(port);
    }

    public double getAxis(Axis axis) {
        switch (axis) {
            // invert Y axes
            case LeftY:
            case RightY:
                return -joystick.getRawAxis(axis.ordinal());
            default:
                return joystick.getRawAxis(axis.ordinal());
        }
    }

    public boolean getButtonPressed(Button button) {
        return joystick.getRawButton(button.ordinal());
    }

    public boolean getPOVPressed(POV direction) {
        return joystick.getPOV(0) == direction.heading();
    }
}
