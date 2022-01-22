package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
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
    private MotorController motorL, motorR;
    private Encoder encoderL, encoderR;
    private AHRS gyro;
    private SimpleMotorFeedforward feedforward;

    // odometry
    private DifferentialDriveKinematics kinematics;
    public DifferentialDriveWheelSpeeds wheelSpeeds;
    public ChassisSpeeds chassisSpeeds;
    private DifferentialDriveOdometry odometry;
    public Pose2d pose;

    private int ks, kv;
    private static final int ERR_TOLERANCE = 5, DERIV_TOLERANCE = 10;
    private double quickStopAccumulator = 0.0;

    public Drivetrain(
        MotorController motorL,
        MotorController motorR,
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
        odometry = new DifferentialDriveOdometry(
            new Rotation2d(Math.toRadians(-gyro.getAngle()))
        );

        // ks = Constants.NEO.ks;
        // kv = Constants.NEO.kv;
        // feedforward = new SimpleMotorFeedforward(ks, kv);

        this.gyro.reset();
        this.encoderL.reset();
        this.encoderR.reset();
    }

    private double clamp(double n, double min, double max) {
        return Math.max(Math.min(n, min), max);
    }

    public void arcadeDrive(double throttle, double wheel) {
        motorL.set(throttle + wheel);
        motorR.set(throttle - wheel);
    }

    // https://github.com/Team254/FRC-2016-Public/blob/master/src/com/team254/frc2016/CheesyDriveHelper.java
    public void curvatureDrive(double throttle, double wheel) {
        double overPower;
        double angularPower;
        boolean isQuickTurn = stopped();

        if (isQuickTurn) {
            if (Math.abs(throttle) < 0.2) {
                double alpha = 0.1;
                quickStopAccumulator = (1 - alpha) * quickStopAccumulator + alpha * clamp(wheel, -1.0, 1.0) * 2;
            }
            overPower = 1.0;
            angularPower = wheel;
        } else {
            overPower = 0.0;
            angularPower = Math.abs(throttle) * wheel * Constants.CURVATURE_TURN_SENS - quickStopAccumulator;
            if (quickStopAccumulator > 1) {
                quickStopAccumulator -= 1;
            } else if (quickStopAccumulator < -1) {
                quickStopAccumulator += 1;
            } else {
                quickStopAccumulator = 0.0;
            }
        }

        double rightPwm = throttle - angularPower;
        double leftPwm = throttle + angularPower;
        if (leftPwm > 1.0) {
            rightPwm -= overPower * (leftPwm - 1.0);
            leftPwm = 1.0;
        } else if (rightPwm > 1.0) {
            leftPwm -= overPower * (rightPwm - 1.0);
            rightPwm = 1.0;
        } else if (leftPwm < -1.0) {
            rightPwm += overPower * (-1.0 - leftPwm);
            leftPwm = -1.0;
        } else if (rightPwm < -1.0) {
            leftPwm += overPower * (-1.0 - rightPwm);
            rightPwm = -1.0;
        }

        motorL.set(leftPwm);
        motorR.set(rightPwm);
    }

    public void stop() {
        motorL.set(0);
        motorR.set(0);
    }

    public boolean stopped() {
        return motorL.get() == 0 && motorR.get() == 0;
    }

    public double getHeading() {
        return gyro.getAngle();
    }

    public double getAvgEncDistance() {
        return (encoderL.getDistance() + encoderR.getDistance()) / 2;
    }

    public void periodic() {
        wheelSpeeds = new DifferentialDriveWheelSpeeds(encoderL.getRate(), encoderR.getRate());
        chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
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
        SmartDashboard.putNumber("θ", pose.getRotation().getDegrees());

        SmartDashboard.putNumber("vX", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("vY", chassisSpeeds.vyMetersPerSecond);
        // I don't know how badly this will affect performance. leaving it out for now.
        /* SmartDashboard.putNumber("Velocity",
            Math.sqrt(
                Math.pow(chassisSpeeds.vyMetersPerSecond, 2) +
                Math.pow(chassisSpeeds.vxMetersPerSecond, 2)
            )
        );*/
        SmartDashboard.putNumber("vθ", -Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond));
    }
}
