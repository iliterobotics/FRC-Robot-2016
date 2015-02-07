package org.usfirst.frc.team1885.robot.comms;

import java.util.ArrayList;

import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.output.RobotControl;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;

public class DataTelemetryService {
	private RobotControl rc = RobotControl.getInstance();
	private TelemetryMessage tm = new TelemetryMessage();
	private SensorInputControl sic = SensorInputControl.getInstance();

	public void setRelays() {
		ArrayList<Relay> temp = (ArrayList<Relay>) rc.getRelay();
		for (int i = 0; i < temp.size(); i++) {
			if (temp.get(i).get().equals(0))
				tm.setRelays(i, 0.0);
			else if (temp.get(i).get().equals(1))
				tm.setRelays(i, 1.0);
			else if (temp.get(i).get().equals(2))
				tm.setRelays(i, 2.0);
			else if (temp.get(i).get().equals(3))
				tm.setRelays(i, 3.0);
		}
	}

	public void setTalons() {
		ArrayList<Talon> temp = (ArrayList<Talon>) rc.getTalons();
		for (int i = 0; i < temp.size(); i++) {
			tm.setDigitalOutputs(i, temp.get(i).get());
		}
	}

	public void setSolenoids() {
		ArrayList[] temp = (ArrayList[]) rc.getSolenoids();
		ArrayList solenoids = temp[0];
		ArrayList doubleSolenoids = temp[1];
		for (int i = 0; i < solenoids.size(); i++) {
			if (!((Solenoid) solenoids.get(i)).get())
				tm.setSolenoids(i, 0);
			else
				tm.setSolenoids(i, 1);
		}
		for (int i = 0; i < doubleSolenoids.size(); i++) {
			if (((DoubleSolenoid) doubleSolenoids.get(i)).get().equals(0))
				tm.setSolenoids(i + solenoids.size(), 0);
			else if (((DoubleSolenoid) doubleSolenoids.get(i)).get().equals(1))
				tm.setSolenoids(i + solenoids.size(), 1);
			else
				tm.setSolenoids(i + solenoids.size(), 2);
		}
	}

	public void setDigitalInputs() {
		ArrayList[] temp = sic.getDigitalInputs();
		ArrayList DI = temp[0];
		ArrayList encoder = temp[1];
		for (int i = 0; i < DI.size(); i++) {
			if (((DigitalInput) DI.get(i)).get())
				tm.setDigitalInputs(i, 1);
			else
				tm.setDigitalInputs(i, 0);
		}
		for (int i = 0; i < encoder.size(); i++) {
			tm.setDigitalInputs(i + DI.size(),
					((Encoder) (encoder.get(i))).get());
		}
	}

}
