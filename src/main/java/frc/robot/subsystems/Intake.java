package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem {
    private IMotorController bottom, top;

    public Intake(IMotorController bottom, IMotorController top) {
        this.bottom = bottom;
        this.top = top;
    }

    public void run() {
        top.set(ControlMode.PercentOutput, 1.0);
        bottom.set(ControlMode.PercentOutput, 1.0);
    }
}
