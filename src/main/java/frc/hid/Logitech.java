package frc.hid;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Logitech {
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
    private final HashMap<Logitech.Axis, Trigger> axisTriggers;
    private final HashMap<Logitech.Button, JoystickButton> buttonTriggers;
    private final HashMap<Logitech.POVButton, edu.wpi.first.wpilibj2.command.button.POVButton> povTriggers;
    // don't ask

    public Logitech(int port) {
        joystick = new Joystick(port);

        axisTriggers = new HashMap<>();
        for (Logitech.Axis axis : Axis.listed()) {
            axisTriggers.put(axis, new Trigger(() -> getAxisValue(axis) != 0));
        }

        buttonTriggers = new HashMap<>();
        for (Logitech.Button button : Button.listed()) {
            buttonTriggers.put(button, new JoystickButton(joystick, button.ordinal()));
        }

        povTriggers = new HashMap<>();
        for (Logitech.POVButton povButton : Logitech.POVButton.listed()) {
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
                return Constants.JOYSTICK_TRANSFORM.applyAsDouble(-joystick.getRawAxis(axis.ordinal()));
            default:
                return Constants.JOYSTICK_TRANSFORM.applyAsDouble(joystick.getRawAxis(axis.ordinal()));
        }
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
