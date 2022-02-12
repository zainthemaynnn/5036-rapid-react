package frc.math;

import java.util.LinkedList;

public class MovingAverage {
    private int maxLen;
    private LinkedList<Double> values;
    private double average;

    public MovingAverage(int maxLen) {
        this.maxLen = maxLen;
        values = new LinkedList<>();
    }

    public void add(double value) {
        values.add(value);
        if (values.size() > maxLen) {
            values.remove();
        }

        average = 0.0;
        values.forEach(v -> average += v);
        average /= values.size();
    }

    public double avg() {
        return average;
    }
}
