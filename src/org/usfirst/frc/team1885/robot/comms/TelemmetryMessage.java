package org.usfirst.frc.team1885.robot.comms;

public class TelemmetryMessage extends Message{
	Double[] digitalInputs = new Double[20];
	Double[] analogInputs = new Double[10];
	Double[] relays = new Double[4];
	Double[] digitalOutputs = new Double[10];
	
	public Double getDigitalInputs(int index) {
		return digitalInputs[index];
	}
	
	public void setDigitalInputs(int index, Double data) {
		this.digitalInputs[index] = data;
	}
	
	public Double getAnalogInputs(int index) {
		return analogInputs[index];
	}
	
	public void setAnalogInputs(int index, Double data) {
		this.analogInputs[index] = data;
	}
	
	public Double getRelays(int index) {
		return relays[index];
	}
	
	public void setRelays(int index, Double data) {
		this.relays[index] = data;
	}
	
	public Double getDigitalOutputs(int index) {
		return digitalOutputs[index];
	}
	
	public void setDigitalOutputs(int index, Double data) {
		this.digitalOutputs[index] = data;
	}
	
}

/*
	20 DigitalInputs
	10 AnalogInputs
	4 Relays
	10 DigitalOutputs
*/
