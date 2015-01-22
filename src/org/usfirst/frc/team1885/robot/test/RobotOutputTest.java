package org.usfirst.frc.team1885.robot.test;

import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.RobotPneumaticType;
import org.usfirst.frc.team1885.robot.output.RobotControl;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class RobotOutputTest 
{
	RobotControl robotControl = RobotControl.getInstance();
	public RobotOutputTest()
	{
		robotControl.addTalonOutput(RobotMotorType.LEFT_DRIVE, 0);
		robotControl.addTalonOutput(RobotMotorType.LEFT_DRIVE, 1);
		robotControl.addTalonOutput(RobotMotorType.RIGHT_DRIVE, 2);
		robotControl.addTalonOutput(RobotMotorType.RIGHT_DRIVE, 3);		
		robotControl.addPneumaticOutput(RobotPneumaticType.GRABBER_PNEUMATIC, 0, 1);
		robotControl.addPneumaticOutput(RobotPneumaticType.LEFT_SHIFTER_PNEUMATIC, 2, 3);
		robotControl.addPneumaticOutput(RobotPneumaticType.RIGHT_SHIFTER_PNEUMATIC, 4,5);
		robotControl.addTalonOutput(RobotMotorType.TOTE_LIFT, 4);
		robotControl.addTalonOutput(RobotMotorType.RECYCLE_LIFT, 4);
	}
	public void outputTest()
	{
		robotControl.updateDriveSpeed(.25, .25);
		robotControl.updateGrabberPneumatics(DoubleSolenoid.Value.kForward);
		robotControl.updateLeftShifter(DoubleSolenoid.Value.kForward);
		robotControl.updateRightShifter(DoubleSolenoid.Value.kForward);
		robotControl.updateRecycleMotor(.25);
		robotControl.updateToteMotor(.25);
	}
	
}
