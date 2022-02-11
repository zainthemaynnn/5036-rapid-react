package frc.devtools;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// PIDController with extra diagnostics. have fun.
public class PIDTuner extends PIDController {
    private ShuffleboardTab tab;
    private NetworkTableEntry kp, ki, kd, dTolerance, vTolerance;

    public PIDTuner(String id) {
        super(0.0, 0.0, 0.0);

        tab = Shuffleboard.getTab(id + " PID Tuner");
        kp = tab.addPersistent("P", getP()).getEntry();
        ki = tab.addPersistent("I", getI()).getEntry();
        kd = tab.addPersistent("D", getD()).getEntry();
        dTolerance = tab.addPersistent("Displacement tolerance", 0.0).getEntry();
        vTolerance = tab.addPersistent("Velocity tolerance", 0.0).getEntry();

        tab.addNumber("Setpoint", this::getSetpoint);
        tab.addNumber("Displacement error", this::getPositionError);
        tab.addNumber("Velocity error", this::getVelocityError);
        // TODO: data points might be getting added too fast, let alone with two of them. will check this out when I can actually test it.
        tab.addNumber("d/t", this::getPositionError).withWidget(BuiltInWidgets.kGraph);
        tab.addNumber("v/t", this::getVelocityError).withWidget(BuiltInWidgets.kGraph);
    }

    private void updateFromShuffleboard() {
        setTolerance(dTolerance.getDouble(0.0), vTolerance.getDouble(0.0));
        setPID(kp.getDouble(0.0), ki.getDouble(0.0), kd.getDouble(0.0));
    }

    @Override
    public double calculate(double measurement) {
        updateFromShuffleboard();
        return super.calculate(measurement);
    }

    @Override
    public double calculate(double measurement, double setpoint) {
        updateFromShuffleboard();
        return super.calculate(measurement, setpoint);
    }
}
