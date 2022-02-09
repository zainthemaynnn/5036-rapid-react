package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Intake implements Subsystem {
    private TalonSRX uno, dos;

    public Intake(TalonSRX uno, TalonSRX dos) {
        this.dos = uno;
        this.uno = dos;
        uno.setNeutralMode(NeutralMode.Brake);
        dos.setNeutralMode(NeutralMode.Brake);
    }

    public void runPercent(double p) {
        dos.set(ControlMode.PercentOutput, p * Constants.INTAKE_POWER);
        uno.set(ControlMode.PercentOutput, p * Constants.INTAKE_POWER);
    }

    public void stop() {
        runPercent(0);
    }
}
