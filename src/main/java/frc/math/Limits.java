package frc.math;

public class Limits {
    public static double clamp(double x, double min, double max) {
        return Math.max(Math.min(x, max), min);
    }

    public static double limit(double x, double b) {
        return clamp(x, -b, b);
    }

    public static double siglim(double x, double min, double max) {
        return clamp(Math.abs(x), min, max) * Math.signum(x);
    }
}
