package org.usfirst.frc.team1885.robot.input;

import java.util.List;

import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class SensorInputControlSRX {
    private static SensorInputControlSRX instance = null;
    private static RobotControlWithSRX rsrx = RobotControlWithSRX.getInstance();
    private PowerDistributionPanel PDP = new PowerDistributionPanel();
    
    public static SensorInputControlSRX getInstance() {
        if (instance == null) {
            instance = new SensorInputControlSRX();
        }
        return instance;
    }
    public void update()
    {
        //Talon Current
        for(CANTalon ct : rsrx.getLeftDrive())
        {
            System.out.println(getCurrent(ct.getDeviceID()) + "This is current for talon ID : " + ct.getDeviceID());
        }
        for(CANTalon ct : rsrx.getRightDrive())
        {
            System.out.println(getCurrent(ct.getDeviceID()) + "This is current for talon ID : " + ct.getDeviceID());
        }
        for(RobotMotorType ct : rsrx.getTalons().keySet())
        {
            System.out.println(getCurrent(rsrx.getTalons().get(ct).getDeviceID()) + "This is current for talon ID : " + rsrx.getTalons().get(ct).getDeviceID());
        }
        //Encoder Position
        for(CANTalon ct : rsrx.getLeftDrive())
        {
            System.out.println(getEncoderPos(ct.getDeviceID()) + "This is encoder position for talon ID : " + ct.getDeviceID());
        }
        for(CANTalon ct : rsrx.getRightDrive())
        {
            System.out.println(getEncoderPos(ct.getDeviceID()) + "This is encoder position for talon ID : " + ct.getDeviceID());
        }
        for(RobotMotorType ct : rsrx.getTalons().keySet())
        {
            System.out.println(getEncoderPos(rsrx.getTalons().get(ct).getDeviceID()) + "This is encoder position for talon ID : " + rsrx.getTalons().get(ct).getDeviceID());
        }
        //Encoder Velocity
        for(CANTalon ct : rsrx.getLeftDrive())
        {
            System.out.println(getEncoderVelocity(ct.getDeviceID()) + "This is encoder velocity for talon ID : " + ct.getDeviceID());
        }
        for(CANTalon ct : rsrx.getRightDrive())
        {
            System.out.println(getEncoderVelocity(ct.getDeviceID()) + "This is encoder velocity for talon ID : " + ct.getDeviceID());
        }
        for(RobotMotorType ct : rsrx.getTalons().keySet())
        {
            System.out.println(getEncoderVelocity(rsrx.getTalons().get(ct).getDeviceID()) + "This is encoder velocity for talon ID : " + rsrx.getTalons().get(ct).getDeviceID());
        }
    }
    public double getCurrent(int channel)
    {
        return PDP.getCurrent(channel);
    }
    public double getPDPTemperature()
    {
        return PDP.getTemperature();
    }    
    public int getEncoderPos(int talonport)
    {
        List<CANTalon> leftDrive = rsrx.getLeftDrive();
        for(CANTalon  ct: leftDrive)
        {
            if(ct.getDeviceID() == talonport)
            {
                return ct.getEncPosition();
            }
        }
        for(CANTalon  ct: rsrx.getRightDrive())
        {
            if(ct.getDeviceID() == talonport)
            {
                return ct.getEncPosition();
            }
        }
        for(RobotMotorType ct: rsrx.getTalons().keySet())
        {
            if(rsrx.getTalons().get(ct).getDeviceID() == talonport)
            {
                return rsrx.getTalons().get(ct).getEncPosition();
            }
        }
        return -1;
    }
    public int getEncoderVelocity(int talonport)
    {
        List<CANTalon> leftDrive = rsrx.getLeftDrive();
        for(CANTalon  ct: leftDrive)
        {
            if(ct.getDeviceID() == talonport)
            {
                return ct.getEncVelocity();
            }
        }
        for(CANTalon  ct: rsrx.getRightDrive())
        {
            if(ct.getDeviceID() == talonport)
            {
                return ct.getEncVelocity();
            }
        }
        for(RobotMotorType  ct: rsrx.getTalons().keySet())
        {
            if(rsrx.getTalons().get(ct).getDeviceID() == talonport)
            {
                return rsrx.getTalons().get(ct).getEncVelocity();
            }
        }
        return -1;
    }
}
