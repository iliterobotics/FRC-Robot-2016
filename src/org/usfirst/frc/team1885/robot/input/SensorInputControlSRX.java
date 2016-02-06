package org.usfirst.frc.team1885.robot.input;

import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.manipulator.AuxArm;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;
import org.usfirst.frc.team1885.robot.sensor.LidarSensor;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class SensorInputControlSRX {
    
    private static SensorInputControlSRX instance = null;
    private static RobotControlWithSRX rsrx;
    private PowerDistributionPanel PDP;
    private LidarSensor ls;
    private BuiltInAccelerometer bia;

    public static final double DEADZONE = 0.1;
    
    private double INITIAL_POT_B_POSITION;
    private double INITIAL_POT_A_POSITION;

    public static SensorInputControlSRX getInstance() {
        if (instance == null) {
            instance = new SensorInputControlSRX();
        }
        return instance;
    }
    protected SensorInputControlSRX() {
        rsrx = RobotControlWithSRX.getInstance();
        PDP = new PowerDistributionPanel();
    }
    public void init(){
        INITIAL_POT_A_POSITION = rsrx.getSensor().get(SensorType.JOINT_A_POTENTIOMETER).getAnalogInPosition() * AuxArm.CONVERSION_FACTOR;
        INITIAL_POT_B_POSITION = rsrx.getSensor().get(SensorType.JOINT_B_POTENTIOMETER).getAnalogInPosition() * AuxArm.CONVERSION_FACTOR;
    }
    public void update() {
        StringBuilder output = new StringBuilder();
//        output.append("\n-LIDAR SENSOR DISTANCE= "
//                + this.getLidarSensor().getDistance());
//        this.getLidarSensor().update();
        output.append("\n-Potentiometer A In Position: "
                + ((this.getAnalogInPosition(SensorType.JOINT_A_POTENTIOMETER) * AuxArm.CONVERSION_FACTOR) ));
        output.append("\n-Potentiometer B In Position: "
                + ((this.getAnalogInPosition(SensorType.JOINT_B_POTENTIOMETER) * AuxArm.CONVERSION_FACTOR) ));
        
        
        DriverStation.reportError(output.toString(), false);
        // System.out.println(this.bia.getX() +" x " + bia.getY() + " y " +
        // bia.getZ() + " z ");
    }
    public double getCurrent(int channel) {
        return PDP.getCurrent(channel);
    }
    public double getPDPTemperature() {
        return PDP.getTemperature();
    }
    public double getAnalogInPosition(SensorType type) {
        return rsrx.getSensor().get(type).getAnalogInPosition();
    }

    public double getAnalogGeneric(SensorType type) {
        return rsrx.getSensor().get(type).getAnalogInRaw();
    }
    public boolean digitalLimitSwitch(SensorType type) {
        return rsrx.getSensor().get(type).isFwdLimitSwitchClosed();
    }
    public int getEncoderPos(SensorType type) {
        return rsrx.getSensor().get(type).getEncPosition();
    }
    public int getEncoderVelocity(SensorType type) {
        return rsrx.getSensor().get(type).getEncVelocity();
    }
    public void addLidarSensor(Port port) {
        ls = new LidarSensor(port);
    }
    public LidarSensor getLidarSensor() {
        return this.ls;

    }
    public void createAccelerometer() {
        bia = new BuiltInAccelerometer();
    }
    public BuiltInAccelerometer getAccelerometer() {
        return bia;
    }
    public double getInitialPotBPostition(){
        return INITIAL_POT_B_POSITION;
    }
    public double getInitialPotAPostition(){
        return INITIAL_POT_A_POSITION;
    }
}
