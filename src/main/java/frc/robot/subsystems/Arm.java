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
    private static final int ARM_UP = 31;
    private static final int ARM_DOWN = 90;
    private static final double DOWN_RANGE = 2.0;

    public Arm(CANSparkMax motor, DigitalInput limitSwitch) {
        this.motor = motor;
        this.limitSwitch = limitSwitch;
        encoder = motor.getEncoder();
        motor.setIdleMode(IdleMode.kBrake);
        SmartDashboard.putNumber("Gravity", 0);
    }

    private double getGravityCompensation() {
        return Constants.GRAVITY * Math.cos(getPosition());
    }

    /*private double getVoltageCompensation() {
        return 12 / RobotContainer.pdp.getVoltage();
    }*/

    public double getPosition() {
        return (encoder.getPosition() * (ARM_DOWN - ARM_UP)) / Constants.ARM_TARGET_ANGLE + ARM_UP;
    }

    public boolean isDown() {
        return Math.abs(getPosition() - ARM_DOWN) < DOWN_RANGE;
    }

    public boolean getLimSwitch() {
        return limitSwitch.get();
    }

    public double armTargetFromPercent(double p) {
        return p * (ARM_DOWN - ARM_UP) + ARM_UP;
    }

    public void setPower(double power) {
        power = -(power + getGravityCompensation());// * getVoltageCompensation();
        motor.set(Math.abs(power) < Constants.ARM_MAX_POWER ? power : Constants.ARM_MAX_POWER * Math.signum(power));
    }

    private static int i = 0;
    public void updateDashboard() {
        SmartDashboard.putNumber("Arm encoder", getPosition());
        SmartDashboard.putBoolean("Limit switch", limitSwitch.get());
        SmartDashboard.putNumber("t", i);
        ++i;
    }

    public void stop() {
        setPower(0.0);
    }
}
