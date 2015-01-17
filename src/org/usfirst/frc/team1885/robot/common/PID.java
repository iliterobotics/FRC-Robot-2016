package org.usfirst.frc.team1885.robot.common;

public class PID {
	private double p, i, d;
	private double error, previousError;
	private double integral;
	
	public PID() {
		error = 0.0;
		integral = 0.0;
	}
	public void setConstraints (double inputP, double inputI, double inputD) {
		p = inputP;
		i = inputI;
		d = inputD;
	}

	public void reset() {
		error = 0;
		previousError = 0;
		integral = 0;
	}

	public double getPID (double projectedSpeed, double currentSpeed) {
		previousError = error;
		error = currentSpeed - projectedSpeed;
		
		return (p * getP()) + (i * getI(1.0)) + (d * getD());
	}

	public double getP() {
		return error;
	}

	public double getI (double dt) {
		return integral += error * dt;
	}

	public double getD () {
		return error - previousError;
	}
}


