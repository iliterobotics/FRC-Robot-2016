package org.usfirst.frc.team1885.robot.common;

import edu.wpi.first.wpilibj.DriverStation;

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
        this.scaleOutput = scaleFactor == 0 ? 1.0 : Math.abs(scaleFactor);
    }

    public double getPID(double projectedValue, double currentValue) {
        previousError = error;

        error = projectedValue - currentValue;

        if (scaleOutput == 0) {
            return 0;
        }

        error = error / this.scaleOutput;

        DriverStation.reportError(
                "\nP: " + (p * getP()) + " ::: I: " + (i * getI(1.0))
                        + " ::: D: " + (d * getD()) + "    --- Error: " + error,
                false);

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
