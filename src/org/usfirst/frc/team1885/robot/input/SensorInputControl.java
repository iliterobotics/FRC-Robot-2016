package org.usfirst.frc.team1885.robot.input;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.usfirst.frc.team1885.robot.common.type.RobotPneumaticType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.sensor.LidarSensor;

import com.kauailabs.nav6.frc.IMUAdvanced;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SerialPort;

public class SensorInputControl {
	private static SensorInputControl instance = null;

	private HashMap<SensorType, Encoder> encoders;
	private HashMap<SensorType, DigitalInput> digital_inputs;
	private HashMap<SensorType, LidarSensor> lidar_sensor;
	private IMUAdvanced imu;
	SerialPort serial_port;

	public static final double DEADZONE = 0.1;

	protected SensorInputControl() {
		encoders = new HashMap<SensorType, Encoder>();
		digital_inputs = new HashMap<SensorType, DigitalInput>();
		lidar_sensor = new HashMap<SensorType, LidarSensor>();

	}

	public ArrayList[] getDigitalInputs() {
		ArrayList[] temp = new ArrayList[2];
		ArrayList<DigitalInput> tempDI = new ArrayList<DigitalInput>();
		ArrayList<Encoder> tempEncoder = new ArrayList<Encoder>();
		for (SensorType st : digital_inputs.keySet())
			tempDI.add(digital_inputs.get(st));
		for (SensorType st : encoders.keySet())
			tempEncoder.add(encoders.get(st));
		temp[0] = tempDI;
		temp[1] = tempEncoder;
		return temp;
	}

	public void setUpNAVX(byte rate, edu.wpi.first.wpilibj.SerialPort.Port port) {
		serial_port = new SerialPort(57600, port);
		imu = new IMUAdvanced(serial_port, rate);
	}

	public static SensorInputControl getInstance() {
		if (instance == null) {
			instance = new SensorInputControl();
		}
		return instance;
	}

	public IMUAdvanced getNAVX() {
		return imu;
	}

	public Encoder getEncoder(SensorType sensor_type) {
		return encoders.get(sensor_type);
	}

	public DigitalInput getLimitSwitch(SensorType sensor_type) {
		return digital_inputs.get(sensor_type);
	}

	public static double deadzone(double axis) {
		if (Math.abs(axis) < DEADZONE) {
			return 0;
		}
		return axis;
	}

	public boolean addSensor(SensorType sensor_type, int port) {
		if (sensor_type.name().contains("LIMIT")
				|| sensor_type.name().contains("LINE")) {
			digital_inputs.put(sensor_type, new DigitalInput(port));
		} else {
			return false;
		}
		return true;
	}

	public boolean addSensor(SensorType sensor_type, int port_one, int port_two) {
		if (sensor_type.name().contains("ENCODER")) {
			encoders.put(sensor_type, new Encoder(port_one, port_two));
		} else {
			return false;
		}
		return true;
	}

	public boolean addLidarSensor(Port port) {
		lidar_sensor.put(SensorType.LIDAR, new LidarSensor(port));
		return true;
	}

	public LidarSensor getLidarSensor(SensorType sensor_type) {
		return lidar_sensor.get(sensor_type);
	}

	public int getEncoderTicks(SensorType sensor_type) {
		return encoders.get(sensor_type).get();
	}

	public double getRate(SensorType sensor_type) {
		return encoders.get(sensor_type).getRate();
	}

	public boolean isActive(SensorType sensor_type) {
		return digital_inputs.get(sensor_type).get();
	}
}