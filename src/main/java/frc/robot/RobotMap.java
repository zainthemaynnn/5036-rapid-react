package frc.robot;

public class RobotMap {
    public static enum PWM {
        // example for now
        BACK_MOTOR_LEFT(0),
        BACK_MOTOR_RIGHT(1),
        FRONT_MOTOR_LEFT(2),
        FRONT_MOTOR_RIGHT(3);

        private int port;

        private PWM(int port) {
            this.port = port;
        }

        public int port() {
            return port;
        }
    }

    public static enum Controller {
        DRIVER(0),
        OPERATOR(1);

        private int port;

        private Controller(int port) {
            this.port = port;
        }

        public int port() {
            return port;
        }
    }
}
