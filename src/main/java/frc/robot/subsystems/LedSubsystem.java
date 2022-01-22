package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

// LEDs for driver feedback, programmer like a regular Spark Motor controller and PWM output
// Flash lights when ball is picked up, this is to be detected by detecting a CURRENT spike (momentary increase) on the intake motors
// We will have to see what baseline current is, then see what the spike is when we get a ball. Might also have to do some 
// simple filtering where we ignore current spike on start up (1s ignore) because that also draws large current
// A quick 3 flash when ball is picked up.
// Also start flashing yellow or something when 30 seconds left

public class LedSubsystem implements Subsystem  {

    public LedSubsystem() {

    }

    public void ballPickUpFlash() {
        // A quick 3 flash when ball is picked up.
        
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

}
