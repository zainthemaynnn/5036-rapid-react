package frc.hid;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickButton extends Trigger {
    public JoystickButton(BooleanSupplier active) {
        super(active);
    }
}
