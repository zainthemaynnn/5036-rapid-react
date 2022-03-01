package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Blinkin.BlinkinColor;
import frc.robot.subsystems.Limelight.LEDState;

public class Intake implements Subsystem {
    private TalonSRX uno, dos;
    private Blinkin blinkin;
    private Limelight limelight;

    public Intake(TalonSRX uno, TalonSRX dos, Blinkin blinkin, Limelight limelight) {
        this.dos = uno;
        this.uno = dos;
        this.blinkin = blinkin;
        this.limelight = limelight;
        uno.setNeutralMode(NeutralMode.Brake);
        dos.setNeutralMode(NeutralMode.Brake);
    }

    public void runPercent(double p) {
        uno.set(ControlMode.PercentOutput, p);
        dos.set(ControlMode.PercentOutput, p);
    }

    public void stop() {
        runPercent(0);
    }

    public boolean hasBall() {
        SmartDashboard.putNumber("current", uno.getStatorCurrent());
        return uno.getStatorCurrent() > 45;
    }
}
