package frc.ui;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

// stupid thing isn't sendable. I made the stupid thing sendable.
public class SendableChassisSpeeds extends ChassisSpeeds implements Sendable {
    public SendableChassisSpeeds(ChassisSpeeds speeds) {
        super(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ChassisSpeeds");
        builder.addDoubleProperty("vX", () -> vxMetersPerSecond, null);
        builder.addDoubleProperty("vY", () -> vyMetersPerSecond, null);
        builder.addDoubleProperty("vR", () -> omegaRadiansPerSecond, null);
        builder.setActuator(true);
    }
}