package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.ui.Dashboard;
import frc.ui.SendableChassisSpeeds;

public class Drivetrain implements Subsystem, AutoCloseable {
    // motors/sensors
    private MotorControllerGroup motorL, motorR;
    private RelativeEncoder encoderL, encoderR;
    private AHRS gyro;
    private IdleMode idleMode;

    // odometry
    private SimpleMotorFeedforward feedforward;
    private DifferentialDriveKinematics kinematics;
    private DifferentialDriveWheelSpeeds wheelSpeeds;
    private SendableChassisSpeeds chassisSpeeds;
    private DifferentialDriveOdometry odometry;
    private Pose2d pose;

    public RamseteController ramseteController = new RamseteController();

    private final int ks = 0, kv = 0, ka = 0; // TODO
    private double quickStopAccumulator = 0.0;

    public Drivetrain(
        MotorControllerGroup motorL,
        MotorControllerGroup motorR,
        RelativeEncoder encoderL,
        RelativeEncoder encoderR,
        AHRS gyro
    ) {
        this.motorL = motorL;
        this.motorR = motorR;
        this.encoderL = encoderL;
        this.encoderR = encoderR;
        this.gyro = gyro;

        motorL.setInverted(false);
        motorR.setInverted(true);
        encoderL.setPositionConversionFactor(42);
        encoderR.setPositionConversionFactor(42);

        kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
        odometry = new DifferentialDriveOdometry(
            new Rotation2d(Math.toRadians(-gyro.getAngle()))
        );
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);

        resetOdometry(new Pose2d());
        updateOdometry();
    
        new Dashboard("Drivetrain")
            .add("Left motor", motorL)
            .add("Right motor", motorR)
            .add("Gyro", gyro)
            .add("Velocity", chassisSpeeds);

        idleMode = IdleMode.kBrake;
    }

    private double clamp(double n, double min, double max) {
        return Math.max(Math.min(n, min), max);
    }

    // this is aadi's
    public void arcadeDrive(double throttle, double wheel) {
        motorL.set(throttle + wheel);
        motorR.set(throttle - wheel);
    }

    // https://github.com/Team254/FRC-2016-Public/blob/master/src/com/team254/frc2016/CheesyDriveHelper.java
    public void curvatureDrive(double throttle, double wheel, boolean isQuickTurn) {
        double overPower;
        double angularPower;

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
        motorL.stopMotor();
        motorR.stopMotor();
    }

    public double getHeading() {
        return gyro.getAngle();
    }

    public double getHeadingRate() {
        return gyro.getRate();
    }

    public double getEncL() {
        return encoderL.getPosition() / 22.953;
    }

    public double getEncR() {
        return -encoderR.getPosition() / 22.953;
    }

    public double getEncAvg() {
        return (getEncL() + getEncR()) / 2;
    }

    public Pose2d getPose() {
        return pose;
    }

    public ChassisSpeeds getVelocity() {
        return chassisSpeeds;
    }

    public void updateOdometry() {
        wheelSpeeds = new DifferentialDriveWheelSpeeds(encoderL.getVelocity(), encoderR.getVelocity());
        chassisSpeeds = new SendableChassisSpeeds(kinematics.toChassisSpeeds(wheelSpeeds));
        // please don't ask, cause I don't know
        Pose2d poseMeters = odometry.update(
            new Rotation2d(Math.toRadians(-gyro.getAngle()+90)),
            Units.inchesToMeters(getEncL()),
            Units.inchesToMeters(getEncR())
        );
        pose = new Pose2d(
            Units.metersToInches(poseMeters.getX()),
            Units.metersToInches(poseMeters.getY()),
            poseMeters.getRotation().minus(new Rotation2d(Math.toRadians(90)))
        );
    }

    public void resetOdometry(Pose2d newPose) {
        gyro.reset();
        encoderL.setPosition(0);
        encoderR.setPosition(0);
        odometry.resetPosition(newPose, gyro.getRotation2d());
    }

    @Override
    public void close() {
        motorL.close();
        motorR.close();
        gyro.close();
    }
}
