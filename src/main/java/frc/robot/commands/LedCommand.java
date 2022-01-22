package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LedSubsystem;

public class LedCommand implements Command {

    private final LedSubsystem m_ledSubsystem;

    public LedCommand(LedSubsystem ledSubsystem) {
        m_ledSubsystem = ledSubsystem;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return null;
    }
    
}
