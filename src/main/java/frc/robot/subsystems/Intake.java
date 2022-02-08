package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Intake implements Subsystem {
    private TalonSRX bottom, top;

    public Intake(TalonSRX top, TalonSRX bottom) {
        this.top = top;
        this.bottom = bottom;
        top.setNeutralMode(NeutralMode.Brake);
        bottom.setNeutralMode(NeutralMode.Brake);
    }

    public void runPercent(double p) {
        top.set(ControlMode.PercentOutput, p * Constants.INTAKE_POWER);
        bottom.set(ControlMode.PercentOutput, p * Constants.INTAKE_POWER);
    }

    public void stop() {
        runPercent(0);
    }
}
