package frc.hid;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class PS4Controller extends GenericHID {
    private static final double DEFAULT_DEADBAND = .02;
    private final edu.wpi.first.wpilibj.PS4Controller controller;

    public final JoystickAxis leftStickX, leftStickY, rightStickX, rightStickY, L2, R2;
    public final JoystickButton triangle, circle, cross, square, L1, R1, L3, R3, share, options, home, touchpad;
    public final POVButton up, upRight, right, downRight, down, downLeft, left, upLeft;

    public PS4Controller(int port, double deadband) {
        super(port);
        controller = new edu.wpi.first.wpilibj.PS4Controller(port);

        leftStickX = new JoystickAxis(controller::getLeftX, deadband, false);
        leftStickY = new JoystickAxis(controller::getLeftY, deadband, true);
        rightStickX = new JoystickAxis(controller::getRightX, deadband, false);
        rightStickY = new JoystickAxis(controller::getRightY, deadband, true);
        L2 = new JoystickAxis(controller::getL2Axis, deadband);
        R2 = new JoystickAxis(controller::getR2Axis, deadband);
        
        triangle = new JoystickButton(controller::getTriangleButton);
        circle = new JoystickButton(controller::getCircleButton);
        cross = new JoystickButton(controller::getCrossButton);
        square = new JoystickButton(controller::getSquareButton);
        L1 = new JoystickButton(controller::getL1Button);
        R1 = new JoystickButton(controller::getR1Button);
        L3 = new JoystickButton(controller::getL3Button);
        R3 = new JoystickButton(controller::getR3Button);
        share = new JoystickButton(controller::getSquareButton);
        options = new JoystickButton(controller::getOptionsButton);
        home = new JoystickButton(controller::getPSButton);
        touchpad = new JoystickButton(controller::getTouchpad);

        up = new POVButton(controller, 0);
        upRight = new POVButton(controller, 45);
        right = new POVButton(controller, 90);
        downRight = new POVButton(controller, 135);
        down = new POVButton(controller, 180);
        downLeft = new POVButton(controller, 225);
        left = new POVButton(controller, 270);
        upLeft = new POVButton(controller, 315);
    }

    public PS4Controller(int port) {
        this(port, DEFAULT_DEADBAND);
    }
}
