package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Intake implements Subsystem {
    private IMotorController bottom, top;

    public Intake(IMotorController top, IMotorController bottom) {
        this.top = top;
        this.bottom = bottom;
    }

    public void run() {
        top.set(ControlMode.PercentOutput, Constants.INTAKE_POWER);
        bottom.set(ControlMode.PercentOutput, Constants.INTAKE_POWER);
    }

    public void periodic() {

    }
}
