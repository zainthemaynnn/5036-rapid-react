package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.RamseteController;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.math.Limits;
import frc.robot.Constants;
import frc.ui.Dashboard;
import frc.ui.SendableChassisSpeeds;

public class Drivetrain implements Subsystem, AutoCloseable {
    // motors/sensors
    private CANSparkMax l1, l2, r1, r2;
    private List<CANSparkMax> motors;
    private RelativeEncoder encoderL, encoderR;
    private AHRS gyro;

    // odometry
    private DifferentialDriveKinematics kinematics;
    private DifferentialDriveWheelSpeeds wheelSpeeds;
    private SendableChassisSpeeds chassisSpeeds;
    private DifferentialDriveOdometry odometry;
    private double prevDist = 0.0;
    private double velocity = 0.0;
    private Pose2d pose;

    private double omega0 = 0.0, omega1 = 0.0;

    public RamseteController ramseteController = new RamseteController();

    private double quickStopAccumulator = 0.0;

    public Drivetrain(
        CANSparkMax l1,
        CANSparkMax l2,
        CANSparkMax r1,
        CANSparkMax r2,
        RelativeEncoder encoderL,
        RelativeEncoder encoderR,
        AHRS gyro
    ) {
        this.l1 = l1;
        this.l2 = l2;
        this.r1 = r1;
        this.r2 = r2;
        motors = List.of(l1, l2, r1, r2);

        this.encoderL = encoderL;
        this.encoderR = encoderR;
        this.gyro = gyro;

        l1.setInverted(false);
        l2.follow(l1);
        r1.setInverted(true);
        r2.follow(r1);

        motors.forEach(m -> {
            m.enableVoltageCompensation(12.0);
            m.setSmartCurrentLimit(40);
            m.burnFlash();
        });

        encoderL.setPositionConversionFactor(42);
        encoderR.setPositionConversionFactor(42);

        kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
        odometry = new DifferentialDriveOdometry(
            new Rotation2d(Math.toRadians(-gyro.getAngle()))
        );

        resetOdometry(new Pose2d());
        updateOdometry();
    
        new Dashboard("Drivetrain")
            .add("Gyro", gyro)
            .add("Velocity", chassisSpeeds);
    }

    // this is aadi's
    public void arcadeDrive(double throttle, double wheel) {
        l1.set(throttle + wheel);
        r1.set(throttle - wheel);
    }

    // https://github.com/Team254/FRC-2016-Public/blob/master/src/com/team254/frc2016/CheesyDriveHelper.java
    public void curvatureDrive(double throttle, double wheel, boolean isQuickTurn) {
        double overPower;
        double angularPower;

        if (isQuickTurn) {
            if (Math.abs(throttle) < 0.2) {
                double alpha = 0.1;
                quickStopAccumulator = (1 - alpha) * quickStopAccumulator + alpha * Limits.lim(wheel, 1.0) * 2;
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

        l1.set(leftPwm);
        r1.set(rightPwm);
    }

    public void stop() {
        motors.forEach(CANSparkMax::stopMotor);
    }

    public double getAngle() {
        return gyro.getAngle();
    }

    public double getAngularVelocity() {
        var v = gyro.getRate();
        return gyro.getRate();
    }

    public double getEncL() {
        return encoderL.getPosition() / 22.953;
    }

    public double getEncR() {
        return encoderR.getPosition() / 22.953;
    }

    public double getEncPosition() {
        return (getEncL() + getEncR()) / 2;
    }

    public double getEncVelocity() {
        return (encoderL.getVelocity() / 22.953 + encoderR.getVelocity() / 22.953) / 2;
    }

    public Pose2d getPose() {
        return pose;
    }

    public double getVelocity() {
        return velocity;
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
        velocity = (getEncPosition() - prevDist) / 0.02;
        prevDist = getEncPosition();
        SmartDashboard.putNumber("enc", getEncPosition());
    }

    public void resetOdometry(Pose2d newPose) {
        gyro.reset();
        encoderL.setPosition(0);
        encoderR.setPosition(0);
        odometry.resetPosition(newPose, gyro.getRotation2d());
    }

    public void resetOdometry() {
        resetOdometry(new Pose2d());
    }

    public void setRampRate(double rampRate) {
        motors.forEach(m -> m.setOpenLoopRampRate(rampRate));
    }

    public void setIdleMode(IdleMode mode) {
        motors.forEach(m -> m.setIdleMode(mode));
    }

    @Override
    public void close() {
        motors.forEach(CANSparkMax::close);
        gyro.close();
    }
}
