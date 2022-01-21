package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.RobotMap;
import frc.robot.Constants;

public class Drivetrain implements Subsystem {
    public MotorControllerGroup motorL, motorR;
    public Encoder encoderL, encoderR;
    public AHRS gyro;
    public DifferentialDrive drive;
    public PIDController pidL, pidR;
    public SimpleMotorFeedforward feedforward;
    public DifferentialDrivePoseEstimator poseEstimator;

    private int ks, kv;
    private int kp, ki, kd;
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
        // ks = Constants.NEO.ks;
        // kv = Constants.NEO.kv;
        // feedforward = new SimpleMotorFeedforward(ks, kv);
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

    public void stop() {
        motorL.set(0);
        motorR.set(0);
    }

    public void periodic() {

    }

    public void updateDashboard() {
        SmartDashboard.putNumber("L", motorL.get());
        SmartDashboard.putNumber("R", motorR.get());

        SmartDashboard.putNumber("X", gyro.getPitch());
        SmartDashboard.putNumber("Y", gyro.getYaw());
        SmartDashboard.putNumber("Z", gyro.getRoll());
    }
}
