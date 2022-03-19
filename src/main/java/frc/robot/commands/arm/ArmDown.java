package frc.robot.commands.arm;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class ArmDown extends RunCommand {
    public ArmDown(Arm arm) {
        super(
            () -> {
                if (!arm.isDown()) {
                    arm.setPower(.6);
                } else {
                    arm.setPower(.06);
                }
            },
            arm
        );
    }
}
