package frc.robot;

public class RobotMap {
    public static enum PWM {
        // example for now
        BackMotorLeft(0),
        BackMotorRight(1),
        FrontMotorLeft(2),
        FrontMotorRight(3);

        int port;

        PWM(int port) {
            this.port = port;
        }

        int getPort() {
            return port;
        }
    }
}
