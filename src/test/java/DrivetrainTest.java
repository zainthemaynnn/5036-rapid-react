import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.junit.After;
import org.junit.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainTest {
    private final Pose2d ERR_BOUNDS = new Pose2d(0.1, 0.1, new Rotation2d(0.5));
    private Drivetrain drivetrain;

    public DrivetrainTest() {
        CANSparkMax
            l1 = new CANSparkMax(RobotMap.CAN.BACK_MOTOR_LEFT.id(), MotorType.kBrushless),
            l2 = new CANSparkMax(RobotMap.CAN.FRONT_MOTOR_LEFT.id(), MotorType.kBrushless),
            r1 = new CANSparkMax(RobotMap.CAN.BACK_MOTOR_RIGHT.id(), MotorType.kBrushless),
            r2 = new CANSparkMax(RobotMap.CAN.FRONT_MOTOR_RIGHT.id(), MotorType.kBrushless);

        drivetrain = new Drivetrain(
            new MotorControllerGroup(l1, l2),
            new MotorControllerGroup(r1, r2),
            l1.getEncoder(),
            r1.getEncoder(),
            new AHRS(SPI.Port.kMXP)
        );
    }

    @Test
    public void zeroed() {
        var pose = drivetrain.getPose();
        assert Math.abs(pose.getX()) < ERR_BOUNDS.getX();
        assert Math.abs(pose.getY()) < ERR_BOUNDS.getY();
        assert Math.abs(pose.getRotation().getDegrees()) < ERR_BOUNDS.getRotation().getDegrees();
    }

    @After
    public void close() {
        drivetrain.close();
    }
}
