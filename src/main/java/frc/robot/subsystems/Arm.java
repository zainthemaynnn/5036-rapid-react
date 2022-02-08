package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Arm implements Subsystem {
    private MotorController motor;
    private RelativeEncoder encoder;
    /*private static final double ENCODER_DISTANCE_PER_PULSE =
        (Math.PI * 2) / Constants.ENCODER_RESOLUTION;*/
    private static final int ARM_UP = 31;
    private static final int ARM_DOWN = 90;

    public Arm(CANSparkMax motor) {
        this.motor = motor;
        encoder = motor.getEncoder();
        motor.setIdleMode(IdleMode.kBrake);
        encoder.setPositionConversionFactor((ARM_DOWN - ARM_UP) / Constants.ARM_TARGET_ANGLE);
        SmartDashboard.putNumber("Gravity", 0);
    }

    private double getGravityCompensation() {
        return Constants.GRAVITY * Math.cos(getPosition());
    }

    /*private double getVoltageCompensation() {
        return 12 / RobotContainer.pdp.getVoltage();
    }*/

    public double getPosition() {
        return -encoder.getPosition() + ARM_UP;
    }

    public double armTargetFromPercent(double p) {
        return p * (ARM_UP + (ARM_DOWN - ARM_UP));
    }

    public void setPower(double power) {
        power = (power + getGravityCompensation());// * getVoltageCompensation();
        motor.set(Math.abs(power) < Constants.ARM_MAX_POWER ? power : Constants.ARM_MAX_POWER * Math.signum(power));
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Arm encoder", getPosition());
    }

    public void stop() {
        motor.set(0.0);
    }
}
