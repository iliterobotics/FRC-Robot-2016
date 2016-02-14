package org.usfirst.frc.team1885.robot.output;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;

public class RobotControlWithSRX 
{
	public static RobotControlWithSRX instance;
	private List<CANTalon> leftDrive;
	private List<CANTalon> rightDrive;
	private Map<RobotMotorType, CANTalon> talons = new HashMap<RobotMotorType, CANTalon>();
	public static synchronized RobotControlWithSRX getInstance() {
		if (instance == null) {
			instance = new RobotControlWithSRX();
		}
		return instance;
	}
	protected RobotControlWithSRX()
	{
		leftDrive = new ArrayList<CANTalon>();
		rightDrive = new ArrayList<CANTalon>();
	}
	public void addTalonOutput(RobotMotorType type, int port) {
		if (type == RobotMotorType.LEFT_DRIVE) {
			leftDrive.add(new CANTalon(port));
		} 
		else if (type == RobotMotorType.RIGHT_DRIVE) {
			// add to right motor
			rightDrive.add(new CANTalon(port));
		}
		else
		{
		    talons.put(type, new CANTalon(port));
		}
	}
	public void updateDriveSpeed(double leftspeed, double rightspeed) {
		for (CANTalon leftMotor : leftDrive) {
			leftMotor.set(-leftspeed);
			//System.out.println(leftMotor.getOutputVoltage() + "Voltage");
		}
		for (CANTalon rightMotor : rightDrive) {
			rightMotor.set(rightspeed);
		}
	}
	public void updateShooterTilt(double tiltSpeed) {
	    if (tiltSpeed < .1) {
	        tiltSpeed = -tiltSpeed;
	        talons.get(RobotMotorType.SHOOTER_TILT).set(tiltSpeed);
	    }
	    else if (tiltSpeed > .1) {
	        tiltSpeed = -tiltSpeed;
	        talons.get(RobotMotorType.SHOOTER_TILT).set(tiltSpeed * .3);
	    }
	    else {
	        talons.get(RobotMotorType.SHOOTER_TILT).set(-.7);
	    }
    }
	public void updateShooterTwist(double twistSpeed) {
        talons.get(RobotMotorType.SHOOTER_TWIST).set(-twistSpeed);
    }
    public void updateFlywheelShooter(double flywheelSpeedLeft,
            double flywheelSpeedRight) {
            talons.get(RobotMotorType.FLYWHEEL_LEFT).set(flywheelSpeedLeft);
            talons.get(RobotMotorType.FLYWHEEL_RIGHT).set(flywheelSpeedRight);
    }
    
	public List<CANTalon> getLeftDrive()
	{
	    return leftDrive;
	}
	public List<CANTalon> getRightDrive()
	{
	    return rightDrive;
	}
	public Map<RobotMotorType, CANTalon> getTalons()
	{
	    return this.talons;
	}
    public void updateIntakeMotors(double intakeLeftSpeed, double intakeRightSpeed) {
        // TODO Auto-generated method stub
        
    }
}
