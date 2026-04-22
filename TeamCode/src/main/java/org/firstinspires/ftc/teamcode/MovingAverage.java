package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class MovingAverage {
    private final double[] values;
    int index = 0;
    int count = 0;
    double sum = 0;
    private final int windowSize;

    public MovingAverage(int windowSize){
        this.values = new double[windowSize];
        this.windowSize = windowSize;
    }

    public double add(double newValue){
        sum -= values[index];
        values[index] = newValue;
        sum += newValue;
        index = (index + 1) % windowSize;
        if (count < values.length){
            count ++;
        }
        return sum / count;
    }

}
