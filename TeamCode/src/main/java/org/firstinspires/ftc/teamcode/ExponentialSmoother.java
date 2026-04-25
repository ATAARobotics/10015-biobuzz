package org.firstinspires.ftc.teamcode;

public class ExponentialSmoother {
    private final double alpha;
    private double smoothedValue;
    private boolean initialized;

    public ExponentialSmoother(double alpha){
        this.alpha = alpha;
    }

    public double addValue(double newValue){
        if (!initialized){
            smoothedValue = newValue;
            initialized = true;
        }
        else {
            smoothedValue = (alpha * newValue) + ((1 - alpha) * smoothedValue);
        }
        return smoothedValue;
    }
}
