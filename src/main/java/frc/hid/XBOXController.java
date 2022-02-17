package frc.hid;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class XBOXController extends GenericHID {
    private static final double DEFAULT_DEADBAND = .02;
    private final XboxController controller;

    public final JoystickAxis leftStickX, leftStickY, rightStickX, rightStickY, leftTrigger, rightTrigger;
    public final JoystickButton Y, B, A, X, leftBumper, rightBumper, leftStickButton, rightStickButton, start;
    public final POVButton up, upRight, right, downRight, down, downLeft, left, upLeft;

    public XBOXController(int port, double deadband) {
        super(port);
        controller = new XboxController(port);

        leftStickX = new JoystickAxis(controller::getLeftX, deadband, false);
        leftStickY = new JoystickAxis(controller::getLeftY, deadband, true);
        rightStickX = new JoystickAxis(controller::getRightX, deadband, false);
        rightStickY = new JoystickAxis(controller::getRightY, deadband, true);
        leftTrigger = new JoystickAxis(controller::getLeftTriggerAxis, deadband);
        rightTrigger = new JoystickAxis(controller::getRightTriggerAxis, deadband);
        
        Y = new JoystickButton(controller::getYButton);
        B = new JoystickButton(controller::getBButton);
        A = new JoystickButton(controller::getAButton);
        X = new JoystickButton(controller::getXButton);
        leftBumper = new JoystickButton(controller::getLeftBumper);
        rightBumper = new JoystickButton(controller::getRightBumper);
        leftStickButton = new JoystickButton(controller::getLeftStickButton);
        rightStickButton = new JoystickButton(controller::getRightStickButton);
        start = new JoystickButton(controller::getStartButton);

        up = new POVButton(controller, 0);
        upRight = new POVButton(controller, 45);
        right = new POVButton(controller, 90);
        downRight = new POVButton(controller, 135);
        down = new POVButton(controller, 180);
        downLeft = new POVButton(controller, 225);
        left = new POVButton(controller, 270);
        upLeft = new POVButton(controller, 315);
    }

    public XBOXController(int port) {
        this(port, DEFAULT_DEADBAND);
    }
}
