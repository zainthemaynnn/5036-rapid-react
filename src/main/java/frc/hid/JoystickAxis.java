package frc.hid;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickAxis extends Trigger {
    private DoubleSupplier valueGetter;
    private double deadband;

    public JoystickAxis(DoubleSupplier valueGetter, double deadband, boolean inverted) {
        super();
        this.valueGetter = inverted ? () -> -valueGetter.getAsDouble() : valueGetter;
        this.deadband = deadband;
    }

    public JoystickAxis(DoubleSupplier valueGetter, double deadband) {
        this(valueGetter, deadband, false);
    }

    @Override
    public boolean get() {
        return this.value() != 0;
    }

    private double deadbandFilter(double v) {
        return Math.abs(v) > deadband ? v : 0;
    }

    public double value() {
        return deadbandFilter(valueGetter.getAsDouble());
    }
}
