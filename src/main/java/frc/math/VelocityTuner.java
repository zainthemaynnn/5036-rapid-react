package frc.math;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VelocityTuner {
    private PIDController pPid, vPid;
    private DoubleSupplier position, velocity;
    private DoubleConsumer output;

    public VelocityTuner(DoubleSupplier position, DoubleSupplier velocity, DoubleConsumer output) {
        this.position = position;
        this.velocity = velocity;
        this.output = output;
        pPid = new PIDController(0, 0, 0);
        vPid = new PIDController(0, 0, 0);
        SmartDashboard.putNumber("P", 0);
        SmartDashboard.putNumber("I", 0);
        SmartDashboard.putNumber("D", 0);
        SmartDashboard.putNumber("vP", 0);
        SmartDashboard.putNumber("vI", 0);
        SmartDashboard.putNumber("vD", 0);
        SmartDashboard.putNumber("velocity", 0);
        SmartDashboard.putNumber("dv", 0);
        SmartDashboard.putNumber("position", 0);
        SmartDashboard.putNumber("dp", 0);
    }

    public void driveVelocity(double target) {
        vPid.setPID(SmartDashboard.getNumber("vP", 0), SmartDashboard.getNumber("vI", 0), SmartDashboard.getNumber("vD", 0));
        output.accept(vPid.calculate(velocity.getAsDouble(), target));
        SmartDashboard.putNumber("velocity", velocity.getAsDouble());
        SmartDashboard.putNumber("dv", vPid.getPositionError());
    }

    public void drivePosition(double target) {
        pPid.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
        vPid.setPID(SmartDashboard.getNumber("vP", 0), SmartDashboard.getNumber("vI", 0), SmartDashboard.getNumber("vD", 0));
        output.accept(
            vPid.calculate(
                velocity.getAsDouble(),
                pPid.calculate(
                    position.getAsDouble(),
                    target
                )
            )
        );
        SmartDashboard.putNumber("velocity", velocity.getAsDouble());
        SmartDashboard.putNumber("dv", vPid.getPositionError());
        SmartDashboard.putNumber("position", position.getAsDouble());
        SmartDashboard.putNumber("dp", pPid.getPositionError());
    }
}
