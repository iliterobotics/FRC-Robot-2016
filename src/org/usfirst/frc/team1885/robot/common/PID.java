package org.usfirst.frc.team1885.robot.common;

public class PID {
    private final double p;
    private final double i;
    private final double d;
    private double error, previousError;
    private double integral;
    private double scaleOutput;

    public PID(double inputP, double inputI, double inputD) {
        p = inputP;
        i = inputI;
        d = inputD;
        error = 0.0;
        integral = 0.0;
        scaleOutput = 1.0;
    }

    public void reset() {
        error = 0;
        previousError = 0;
        integral = 0;
    }

    public void setScalingValue(double scaleFactor) {
        this.scaleOutput = Math.abs(scaleFactor);
    }

    public double getPID(double projectedValue, double currentValue) {
        previousError = error;

        error = projectedValue - currentValue;

        error = error / this.scaleOutput;

        double output = (p * getP()) + (i * getI(1.0)) + (d * getD());

        return output;
    }

    public double getP() {
        return error;
    }

    public double getI(double dt) {
        return integral += error * dt;
    }

    public double getD() {
        return error - previousError;
    }
}
