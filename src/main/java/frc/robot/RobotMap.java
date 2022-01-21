package frc.robot;

public class RobotMap {
    public static enum PWM {
        // example for now
        BackMotorLeft(0),
        BackMotorRight(1),
        FrontMotorLeft(2),
        FrontMotorRight(3);

        private int port;

        private PWM(int port) {
            this.port = port;
        }

        public int port() {
            return port;
        }
    }

    public static enum Controller {
        Driver(0),
        Operator(1);

        private int port;

        private Controller(int port) {
            this.port = port;
        }

        public int port() {
            return port;
        }
    }
}
