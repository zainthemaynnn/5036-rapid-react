package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Joystick;

public class Gamepad {
    public static enum Axis {
        LEFT_X,
        LEFT_Y,
        L2,
        R2,
        RIGHT_X,
        RIGHT_Y;
    }

    public static enum JoystickButton {
        _UNUSED,
        GREEN,
        RED,
        BLUE,
        YELLOW,
        L1,
        L2,
        BACK,
        START;
    
        static final JoystickButton[] listed = JoystickButton.values();

        public static JoystickButton[] listed() {
            return listed;
        }
    }

    public static enum POVButton {
        UP,
        UP_RIGHT,
        RIGHT,
        DOWN_RIGHT,
        DOWN,
        DOWN_LEFT,
        LEFT,
        UP_LEFT;

        private int heading;
        static final POVButton[] listed = POVButton.values();

        private POVButton() {
            heading = ordinal() * 45;
        }

        public int heading() {
            return heading;
        }

        public static POVButton[] listed() {
            return listed;
        }
    }

    private final Joystick joystick;
    private final HashMap<Gamepad.JoystickButton, edu.wpi.first.wpilibj2.command.button.JoystickButton> buttons;
    private final HashMap<Gamepad.POVButton, edu.wpi.first.wpilibj2.command.button.POVButton> povButtons;
    // don't ask

    public Gamepad(int port) {
        joystick = new Joystick(port);

        buttons = new HashMap<>();
        for (Gamepad.JoystickButton button : Gamepad.JoystickButton.listed()) {
            buttons.put(button, new edu.wpi.first.wpilibj2.command.button.JoystickButton(joystick, button.ordinal()));
        }

        povButtons = new HashMap<>();
        for (Gamepad.POVButton povButton : Gamepad.POVButton.listed()) {
            povButtons.put(povButton, new edu.wpi.first.wpilibj2.command.button.POVButton(joystick, povButton.heading()));
        }
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

    public double getAxis(Axis axis, double deadband) {
        double v = getAxis(axis);
        return Math.abs(v) > deadband ? v : 0;
    }

    public edu.wpi.first.wpilibj2.command.button.JoystickButton getButton(Gamepad.JoystickButton button) {
        return buttons.get(button);
    }

    public edu.wpi.first.wpilibj2.command.button.POVButton getPOV(Gamepad.POVButton povButton) {
        return povButtons.get(povButton);
    }

    public int getPOVHeading() {
        return joystick.getPOV();
    }
}
