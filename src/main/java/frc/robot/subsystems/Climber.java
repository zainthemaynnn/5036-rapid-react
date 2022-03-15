package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class Climber implements Subsystem {
    private VictorSPX uno, dos;
    private double POWER_UP = +1.00, POWER_DOWN = -1.00;

    public Climber(VictorSPX uno, VictorSPX dos) {
        this.uno = uno;
        this.dos = dos;
        uno.setInverted(true);
        dos.setInverted(true);
    }

    public void extend() {
        uno.set(ControlMode.PercentOutput, POWER_UP);
        dos.set(ControlMode.PercentOutput, POWER_UP);
    }

    public void retract() {
        uno.set(ControlMode.PercentOutput, POWER_DOWN);
        dos.set(ControlMode.PercentOutput, POWER_DOWN);
    }

    public void stop() {
        uno.set(ControlMode.PercentOutput, .00);
        dos.set(ControlMode.PercentOutput, .00);
    }
}
