package org.usfirst.frc.team1885.robot.comms;

import java.util.Arrays;

import org.usfirst.frc.team1885.robot.output.RobotControl;

public class TelemetryMessage extends Message{
	
	private static final long serialVersionUID = 4021050113420280510L;
	private double[] digitalInputs = new double[20];
	private double[] analogInputs = new double[10];
	private double[] relays = new double[4];
	private double[] digitalOutputs = new double[10];
	private double[] solenoids = new double[10]; //Find specific amount later
	
	public double getSolenoids(int index) {
		if(index >= 0 && index < solenoids.length)
			return solenoids[index];
		else
			return -1;
	}

	public void setSolenoids(int index, double data) {
		if(index >= 0 && index < solenoids.length)
			this.solenoids[index] = data;
	}
	
	public double getDigitalInputs(int index) {
		if(index >= 0 && index < digitalInputs.length)
			return digitalInputs[index];
		else
			return -1;
	}
	
	public void setDigitalInputs(int index, double data) {
		if(index >= 0 && index < digitalInputs.length)
			this.digitalInputs[index] = data;
	}
	
	public double getAnalogInputs(int index) {
		if(index >= 0 && index < analogInputs.length)
			return analogInputs[index];
		else
			return -1;
	}
	
	public void setAnalogInputs(int index, double data) {
		if(index >= 0 && index < analogInputs.length)
			this.analogInputs[index] = data;
	}
	
	public double getRelays(int index) {
		if(index >= 0 && index < relays.length)
			return relays[index];
		else
			return -1;
	}
	
	public void setRelays(int index, double data) {
		if(index >= 0 && index < relays.length)
			this.relays[index] = data;
	}
	
	public double getDigitalOutputs(int index) {
		if(index >= 0 && index < digitalOutputs.length)
			return digitalOutputs[index];
		else
			return -1;
	}
	
	public void setDigitalOutputs(int index, double data) {
		if(index >= 0 && index < digitalOutputs.length)
			this.digitalOutputs[index] = data;
	}
	
	public long getSerialID() {
		return serialVersionUID;
	}

	@Override
	public String toString() {
		return "TelemmetryMessage [digitalInputs="
				+ Arrays.toString(digitalInputs) + ", analogInputs="
				+ Arrays.toString(analogInputs) + ", relays="
				+ Arrays.toString(relays) + ", digitalOutputs="
				+ Arrays.toString(digitalOutputs) + "]";
	}
	
}

/*
	20 DigitalInputs
	10 AnalogInputs
	4 Relays
	10 DigitalOutputs
*/
