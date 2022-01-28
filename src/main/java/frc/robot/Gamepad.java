package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Gamepad {
    public static enum Axis {
        LEFT_X,
        LEFT_Y,
        L2,
        R2,
        RIGHT_X,
        RIGHT_Y;

        static final Axis[] listed = Axis.values();

        public static Axis[] listed() {
            return listed;
        }
    }

    public static enum Button {
        _UNUSED,
        GREEN,
        RED,
        BLUE,
        YELLOW,
        L1,
        R1,
        BACK,
        START;
    
        static final Button[] listed = Button.values();

        public static Button[] listed() {
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

        static final POVButton[] listed = POVButton.values();

        public static POVButton[] listed() {
            return listed;
        }

        private int heading;

        private POVButton() {
            heading = ordinal() * 45;
        }

        public int heading() {
            return heading;
        }
    }

    private final Joystick joystick;
    private final HashMap<Gamepad.Axis, Trigger> axisTriggers;
    private final HashMap<Gamepad.Button, JoystickButton> buttonTriggers;
    private final HashMap<Gamepad.POVButton, edu.wpi.first.wpilibj2.command.button.POVButton> povTriggers;
    // don't ask

    public Gamepad(int port) {
        joystick = new Joystick(port);

        axisTriggers = new HashMap<>();
        for (Gamepad.Axis axis : Axis.listed()) {
            axisTriggers.put(axis, new Trigger(() -> getAxisValue(axis, .02) != 0));
        }

        buttonTriggers = new HashMap<>();
        for (Gamepad.Button button : Button.listed()) {
            buttonTriggers.put(button, new JoystickButton(joystick, button.ordinal()));
        }

        povTriggers = new HashMap<>();
        for (Gamepad.POVButton povButton : Gamepad.POVButton.listed()) {
            povTriggers.put(povButton, new edu.wpi.first.wpilibj2.command.button.POVButton(joystick, povButton.heading()));
        }
    }

    public Trigger getAxis(Axis axis) {
        return axisTriggers.get(axis);
    }

    public double getAxisValue(Axis axis) {
        switch (axis) {
            // invert Y axes
            case LEFT_Y:
            case RIGHT_Y:
                return -joystick.getRawAxis(axis.ordinal());
            default:
                return joystick.getRawAxis(axis.ordinal());
        }
    }

    public double getAxisValue(Axis axis, double deadband) {
        double v = getAxisValue(axis);
        return Math.abs(v) > deadband ? v : 0;
    }

    public JoystickButton getButton(Button button) {
        return buttonTriggers.get(button);
    }

    public edu.wpi.first.wpilibj2.command.button.POVButton getPOV(POVButton povButton) {
        return povTriggers.get(povButton);
    }

    public int getPOVHeading() {
        return joystick.getPOV();
    }
}
