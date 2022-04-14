package frc.math;

public class Limits {
    public static double lim(double x, double min, double max) {
        return Math.max(Math.min(x, max), min);
    }

    public static double lim(double x, double b) {
        return lim(x, -b, b);
    }

    public static double siglim(double x, double min, double max) {
        return lim(Math.abs(x), min, max) * Math.signum(x);
    }
}
