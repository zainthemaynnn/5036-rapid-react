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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;

public class Drivetrain implements Subsystem {
    // motors/sensors
    private MotorController motorL, motorR;
    private Encoder encoderL, encoderR;
    private AHRS gyro;

    // odometry
    public SimpleMotorFeedforward feedforward;
    public DifferentialDriveKinematics kinematics;
    public DifferentialDriveWheelSpeeds wheelSpeeds;
    public ChassisSpeeds chassisSpeeds;
    private DifferentialDriveOdometry odometry;
    public Pose2d pose;

    private final int kp = 0, ki = 0, kd = 0; // TODO
    public PIDController pidL = new PIDController(kp, ki, kd);
    public PIDController pidR = new PIDController(kp, ki, kd);
    public RamseteController ramseteController = new RamseteController();

    private final int ks = 0, kv = 0, ka = 0; // TODO
    private double quickStopAccumulator = 0.0;
    private static final double ENCODER_DISTANCE_PER_PULSE =
        Math.PI * 2 * Constants.WHEEL_RADIUS / Constants.ENCODER_RESOLUTION;

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

        motorL.setInverted(false);
        motorR.setInverted(true);
        encoderL.setDistancePerPulse(ENCODER_DISTANCE_PER_PULSE);
        encoderR.setDistancePerPulse(ENCODER_DISTANCE_PER_PULSE);

        kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
        odometry = new DifferentialDriveOdometry(
            new Rotation2d(Math.toRadians(-gyro.getAngle()))
        );

        feedforward = new SimpleMotorFeedforward(ks, kv, ka);

        gyro.reset();
        encoderL.reset();
        encoderR.reset();
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
        boolean isQuickTurn = stopped(); // TODO: is this right? might want to put a threshold > 0.

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

    public void tankDriveVolts(double voltageL, double voltageR) {
        motorL.setVoltage(voltageL);
        motorR.setVoltage(voltageR);
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

    public double getHeadingRate() {
        return gyro.getRate();
    }

    public double getAvgEncDistance() {
        return (encoderL.getDistance() + encoderR.getDistance()) / 2;
    }

    private void updateOdometry() {
        wheelSpeeds = new DifferentialDriveWheelSpeeds(encoderL.getRate(), encoderR.getRate());
        chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
        pose = odometry.update(
            new Rotation2d(Math.toRadians(-gyro.getAngle())),
            encoderL.getDistance(),
            encoderR.getDistance()
        );
    }

    public void resetOdometry(Pose2d newPose) {
        encoderL.reset();
        encoderR.reset();
        odometry.resetPosition(newPose, gyro.getRotation2d());
    }

    private void updateDashboard() {
        SmartDashboard.putNumber("MotorL", motorL.get());
        SmartDashboard.putNumber("MotorR", motorR.get());

        SmartDashboard.putNumber("X", pose.getX());
        SmartDashboard.putNumber("Y", pose.getY());
        SmartDashboard.putNumber("θ", pose.getRotation().getDegrees());

        SmartDashboard.putNumber("vX", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("vY", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("vθ", -Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond));
    }

    public void periodic() {
        updateOdometry();
        updateDashboard();
    }
}
