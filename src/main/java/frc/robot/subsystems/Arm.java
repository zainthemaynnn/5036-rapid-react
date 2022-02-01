package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Arm implements Subsystem {
    private MotorController motor;

    public Arm(MotorController motor) {
        this.motor = motor;
    }

    public void lower() {
        motor.set(Constants.ARM_POWER);
    }

    public void raise() {
        motor.set(-Constants.ARM_POWER);
    }

    public void stop() {
        motor.set(0.0);
    }
}
