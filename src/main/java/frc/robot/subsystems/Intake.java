package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Intake implements Subsystem {
    private TalonSRX uno, dos;
    private PowerDistribution pdp;

    public Intake(TalonSRX uno, TalonSRX dos/*, PowerDistribution pdp*/) {
        this.dos = uno;
        this.uno = dos;
        uno.setNeutralMode(NeutralMode.Brake);
        dos.setNeutralMode(NeutralMode.Brake);
        uno.enableVoltageCompensation(false);
        dos.enableVoltageCompensation(false);
        //this.pdp = pdp;
    }

    public void runPercent(double p) {
        uno.set(ControlMode.PercentOutput, p * Constants.INTAKE_POWER/* * (12 / pdp.getVoltage())*/);
        dos.set(ControlMode.PercentOutput, p * Constants.INTAKE_POWER/* * (12 / pdp.getVoltage())*/);
    }

    public void stop() {
        runPercent(0);
    }

    public boolean hasBall() {
        /*SmartDashboard.putNumber("current", uno.getStatorCurrent());
        SmartDashboard.putNumber("voltage", pdp.getVoltage());
        SmartDashboard.putNumber("comp", 12 / pdp.getVoltage());*/
        return uno.getStatorCurrent() > 45;
    }
}
