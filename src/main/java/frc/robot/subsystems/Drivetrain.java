package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase {
    MotorControllerGroup motorL, motorR;
    Encoder encoderL, encoderR;
    AHRS gyro;
    DifferentialDrive drive;
    PIDController pidL, pidR;

    private static int kp, ki, kd;
    private static final int ERR_TOLERANCE = 5, DERIV_TOLERANCE = 10;

    public Drivetrain(
        MotorControllerGroup motorL,
        MotorControllerGroup motorR,
        Encoder encoderL,
        Encoder encoderR,
        AHRS gyro
    ) {
        this.motorL = motorL;
        this.motorR = motorR;
        this.encoderL = encoderL;
        this.encoderR = encoderR;
        this.gyro = gyro;

        drive = new DifferentialDrive(motorL, motorR);
        kp = ki = kd = 0;
        pidL = new PIDController(kp, ki, kd);
        pidR = new PIDController(kp, ki, kd);

        this.gyro.reset();
        this.encoderL.reset();
        this.encoderR.reset();
    }

    private double clamp(double n, double min, double max) {
        return Math.min(Math.max(n, min), max);
    }

    public double getEncAvg() {
        return (encoderL.getDistance() + encoderR.getDistance()) / 2;
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("L", motorL.get());
        SmartDashboard.putNumber("R", motorR.get());

        SmartDashboard.putNumber("X", gyro.getPitch());
        SmartDashboard.putNumber("Y", gyro.getYaw());
        SmartDashboard.putNumber("Z", gyro.getRoll());
    }

    public static Drivetrain create() {
        Spark LF = new Spark(RobotMap.PWM.FrontMotorLeft.getPort());
        Spark RF = new Spark(RobotMap.PWM.FrontMotorRight.getPort());
        Spark LB = new Spark (RobotMap.PWM.BackMotorLeft.getPort());
        Spark RB = new Spark (RobotMap.PWM.BackMotorRight.getPort());

        return new Drivetrain(List.asList(RF,LF), motorR, encoderL, encoderR, gyro)
    }
}
