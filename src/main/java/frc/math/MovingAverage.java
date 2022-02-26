package frc.math;

import java.util.LinkedList;

public class MovingAverage {
    private int maxLen;
    private LinkedList<Double> values;
    private double average;

    public MovingAverage(int maxLen) {
        this.maxLen = maxLen;
        values = new LinkedList<>();
        average = 0.0;
    }

    public void add(double value) {
        values.add(value);
        if (values.size() > maxLen) {
            var first = values.remove();
            average -= first / values.size();
        }
            
        average += value / values.size();
    }

    public double avg() {
        return average;
    }
}
