package frc.devtools;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// PIDController with extra diagnostics. have fun.
public class PIDTuner extends PIDController {
    private ShuffleboardTab tab;
    private NetworkTableEntry kp, ki, kd, dTolerance, vTolerance;
    private double lastValue = 503.6;

    public PIDTuner(String id, double kp, double ki, double kd) {
        super(kp, ki, kd);

        tab = Shuffleboard.getTab(id + " PID Tuner");
        this.kp = tab.addPersistent("P", getP()).getEntry();
        this.ki = tab.addPersistent("I", getI()).getEntry();
        this.kd = tab.addPersistent("D", getD()).getEntry();
        dTolerance = tab.addPersistent("Displacement tolerance", 0.0).getEntry();
        vTolerance = tab.addPersistent("Velocity tolerance", 0.0).getEntry();

        tab.addNumber("Setpoint", this::getSetpoint);
        tab.addNumber("Prev", () -> lastValue);
        tab.addNumber("Displacement error", this::getPositionError);
        tab.addNumber("Velocity error", this::getVelocityError);
        // TODO: data points might be getting added too fast, let alone with two of them. will check this out when I can actually test it.
        //tab.addNumber("d/t", this::getPositionError).withWidget(BuiltInWidgets.kGraph);
        //tab.addNumber("v/t", this::getVelocityError).withWidget(BuiltInWidgets.kGraph);
        tab.addBoolean("At setpoint?", this::atSetpoint);
    }
    
    public PIDTuner(String id) {
        this(id, 0.0, 0.0, 0.0);
    }

    private void updateFromShuffleboard() {
        setTolerance(dTolerance.getDouble(0.0), vTolerance.getDouble(0.0));
        setPID(kp.getDouble(0.0), ki.getDouble(0.0), kd.getDouble(0.0));
    }

    @Override
    public double calculate(double measurement) {
        lastValue = measurement;
        updateFromShuffleboard();
        return super.calculate(measurement);
    }

    @Override
    public double calculate(double measurement, double setpoint) {
        lastValue = measurement;
        updateFromShuffleboard();
        return super.calculate(measurement, setpoint);
    }
}
