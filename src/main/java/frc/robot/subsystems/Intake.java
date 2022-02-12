package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Intake implements Subsystem {
    private TalonSRX uno, dos;

    private double velocityCompensated(double power) {
        return power;
    }

    public Intake(TalonSRX uno, TalonSRX dos) {
        this.dos = uno;
        this.uno = dos;
        uno.setNeutralMode(NeutralMode.Brake);
        dos.setNeutralMode(NeutralMode.Brake);
    }

    public void runPercent(double p) {
        uno.set(ControlMode.PercentOutput, velocityCompensated(p * Constants.INTAKE_POWER));
        dos.set(ControlMode.PercentOutput, velocityCompensated(p * Constants.INTAKE_POWER));
    }

    public void stop() {
        runPercent(0);
    }
}
