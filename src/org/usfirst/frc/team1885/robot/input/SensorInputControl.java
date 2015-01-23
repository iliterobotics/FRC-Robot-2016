package org.usfirst.frc.team1885.robot.input;

import java.util.HashMap;

import org.usfirst.frc.team1885.robot.common.type.Sensor;
import org.usfirst.frc.team1885.robot.sensor.LidarSensor;

import com.kauailabs.nav6.frc.IMUAdvanced;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Joystick;

public class SensorInputControl
{	
	private static SensorInputControl instance = null;
	
	private HashMap<Sensor, Joystick> joysticks;
	private HashMap<Sensor, Encoder> encoders;
	private HashMap<Sensor, DigitalInput> limit_switches;
	private HashMap<Sensor, LidarSensor> lidar_sensor;
	private IMUAdvanced imu;
	SerialPort serial_port;
	
	public static final double DEADZONE = 0.1;
	protected SensorInputControl()
	{
		joysticks = new HashMap<Sensor, Joystick>();
		encoders = new HashMap<Sensor, Encoder>();
		limit_switches = new HashMap<Sensor, DigitalInput>();
		lidar_sensor = new HashMap<Sensor, LidarSensor>();
		
	}
	public void setUpNAVX(byte rate, edu.wpi.first.wpilibj.SerialPort.Port port)
	{
		serial_port = new SerialPort(57600, port);
		imu = new IMUAdvanced(serial_port, rate);
	}
	public static SensorInputControl getInstance()
	{
		if(instance == null)
		{
			instance = new SensorInputControl();
		}
		return instance;
	}
	public IMUAdvanced getNAVX()
	{
		return imu;
	}
	public Joystick getJoystick(Sensor sensor_type)
	{
		return joysticks.get(sensor_type);
	}
	public Encoder getEncoder(Sensor sensor_type)
    {
    	return encoders.get(sensor_type);
    }
    public DigitalInput getLimitSwitch(Sensor sensor_type)
    {
    	return limit_switches.get(sensor_type);
    }
    public static double deadzone(double axis)
    {
    	if(Math.abs(axis) < DEADZONE)
    	{
    		return 0;
    	}
    	return axis;
    }    
    public boolean addSensor(Sensor sensor_type, int port)
    {
    	if(joysticks.containsKey(sensor_type) || encoders.containsKey(sensor_type) 
    			|| limit_switches.containsKey(sensor_type))
    	{
    		return false;
    	}
    	if(sensor_type.name().contains("JOYSTICK"))
    	{
    		joysticks.put(sensor_type, new Joystick(port));
    	}
    	else if(sensor_type.name().contains("LIMIT"))
    	{
    		limit_switches.put(sensor_type, new DigitalInput(port));
    	}
    	else
    	{
    		return false;
    	}
    	return true;
    }
    public boolean addSensor(Sensor sensor_type, int port_one, int port_two)
    {
    	if(joysticks.containsKey(sensor_type) || encoders.containsKey(sensor_type) 
    			|| limit_switches.containsKey(sensor_type))
    	{
    		return false;
    	}
    	if(sensor_type.name().contains("ENCODER"))
    	{
    		encoders.put(sensor_type, new Encoder(port_one, port_two));
    	}
    	else
    	{
    		return false;
    	}
    	return true;
    }
    public boolean addLidarSensor(Port port)
    {
    	lidar_sensor.put(Sensor.LIDAR , new LidarSensor(port));
    	return true;
    }
    public LidarSensor getLidarSensor(Sensor sensor_type)
    {
    	return lidar_sensor.get(sensor_type);
    }
    public int getEncoderTicks(Sensor sensor_type)
    {
    	return encoders.get(sensor_type).get();
    }
    public double getRate(Sensor sensor_type)
    {
    	return encoders.get(sensor_type).getRate();
    }
    public boolean isLimitSwitch(Sensor sensor_type)
    {
    	return limit_switches.get(sensor_type).get();
    }
}
