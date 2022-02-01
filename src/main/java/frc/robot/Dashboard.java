package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;

public class Dashboard {
    String subsystem;

    public Dashboard(String subsystem) {
        this.subsystem = subsystem;
    }

    public Dashboard add(String name, Sendable obj) {
        SendableRegistry.addLW(obj, subsystem, name);
        return this;
    }
}
