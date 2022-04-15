package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Arm implements Subsystem {
    private MotorController motor;
    private RelativeEncoder encoder;
    private DigitalInput limitSwitch;
    private boolean overridden = false;
    public static final int ARM_UP = 32;
    public static final int ARM_DOWN = 90;
    private static final double DOWN_RANGE = 5.0;
    private static final double UP_RANGE = 5.0;

    public Arm(CANSparkMax motor, DigitalInput limitSwitch) {
        this.motor = motor;
        this.limitSwitch = limitSwitch;
        encoder = motor.getEncoder();
        motor.setIdleMode(IdleMode.kBrake);
    }

    public double getPosition() {
        return (encoder.getPosition() * (ARM_DOWN - ARM_UP)) / Constants.ARM_TARGET_ANGLE + ARM_UP;
    }

    public boolean isDown() {
        return Math.abs(getPosition() - ARM_DOWN) < DOWN_RANGE;
    }

    public boolean isUp() {
        return Math.abs(getPosition() - ARM_UP) < UP_RANGE;
    }

    public void zero() {
        encoder.setPosition(0);
    }

    public double armTargetFromPercent(double p) {
        return p * (ARM_DOWN - ARM_UP) + ARM_UP;
    }

    public void setPower(double power) {
        power = -(power);
        motor.set(power);
    }

    public void setIdleMode(IdleMode idleMode) {
        //motor
    }

    public void override(boolean on) {
        overridden = on;
    }

    public boolean overriden() {
        return overridden;
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Arm encoder", getPosition());
        SmartDashboard.putBoolean("Limit switch", limitSwitch.get());
    }

    public void stop() {
        setPower(0.0);
    }
}
