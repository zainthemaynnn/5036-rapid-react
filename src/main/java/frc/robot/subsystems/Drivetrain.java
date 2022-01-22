package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;

public class Drivetrain implements Subsystem {
    public MotorControllerGroup motorL, motorR;
    public Encoder encoderL, encoderR;
    public AHRS gyro;
    public DifferentialDrive drive;
    public SimpleMotorFeedforward feedforward;

    // odometry
    private DifferentialDriveKinematics kinematics;
    public DifferentialDriveWheelSpeeds wheelSpeeds;
    public ChassisSpeeds chassisSpeeds;
    public DifferentialDriveOdometry odometry;
    public Pose2d pose;

    private int ks, kv;
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

        kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
        wheelSpeeds = new DifferentialDriveWheelSpeeds(encoderL.getRate(), encoderR.getRate());
        chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
        odometry = new DifferentialDriveOdometry(
            new Rotation2d(Math.toRadians(-gyro.getAngle()))
        );

        drive = new DifferentialDrive(motorL, motorR);
        // ks = Constants.NEO.ks;
        // kv = Constants.NEO.kv;
        // feedforward = new SimpleMotorFeedforward(ks, kv);

        this.gyro.reset();
        this.encoderL.reset();
        this.encoderR.reset();
    }

    public void stop() {
        motorL.set(0);
        motorR.set(0);
    }

    public double getAvgEncDistance() {
        return (encoderL.getDistance() + encoderR.getDistance()) / 2;
    }

    public void periodic() {
        pose = odometry.update(
            new Rotation2d(Math.toRadians(-gyro.getAngle())),
            encoderL.getDistance(),
            encoderR.getDistance()
        );

        updateDashboard();
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("MotorL", motorL.get());
        SmartDashboard.putNumber("MotorR", motorR.get());

        SmartDashboard.putNumber("X", pose.getX());
        SmartDashboard.putNumber("Y", pose.getY());
        SmartDashboard.putNumber("Rotation", pose.getRotation().getDegrees());

        SmartDashboard.putNumber("vX", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("vY", chassisSpeeds.vyMetersPerSecond);
        // I don't know how badly this will affect performance. leaving it out for now.
        /* SmartDashboard.putNumber("Velocity",
            Math.sqrt(
                Math.pow(chassisSpeeds.vyMetersPerSecond, 2) +
                Math.pow(chassisSpeeds.vxMetersPerSecond, 2)
            )
        );*/
        SmartDashboard.putNumber("Angular Velocity", -Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond));
    }
}
