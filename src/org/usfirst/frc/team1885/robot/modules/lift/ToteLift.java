package org.usfirst.frc.team1885.robot.modules.lift;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControl;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.modules.Module;
import org.usfirst.frc.team1885.robot.output.RobotControl;

public class ToteLift implements Module{

	private final int toteHeight = 42; /* psuedoheight */
	public static final double DEFAULT_LIFT_SPEED = -.5;
	public static final double DEFAULT_LIFT_SPEED_DOWN = .25;
	private static ToteLift instance;
	private double liftSpeed;
	private MotorState state;
	private PID distanceControlLoop;
	private PID motorPowerControlLoop;
	private double distanceTraveled;
	private double motorPower;
	private final double DEAD_ZONE = .1;
	private boolean isIncrementing;
	private boolean isBraked;

	private boolean isResetting;

	private double liftIncrementHeight = 1210;
	private boolean isDecrementing;

	protected ToteLift() {
		this.state = MotorState.STOP;
		distanceControlLoop = new PID(0.01, 0.00001, 0);
		encoderDistanceControlLoop = new PID(.01, .00001, 0);
		isBraked = false;
		liftSpeed = 0;
		isIncrementing = false;
		isDecrementing = false;
	}

	public static ToteLift getInstance() {
		if (instance == null) {
			instance = new ToteLift();
		}
		return instance;
	}

	public double getDistanceTraveled() {
		return distanceTraveled;
	}

	public void setMotorState(MotorState state) {
		this.state = state;
	}

	public MotorState getMotorState() {
		return state;
	}

	public double getSpeed() {
		return liftSpeed;
	}

	public void updateLift() {

		if ((DriverInputControl.getInstance().getButton(
				RobotButtonType.TOTE_LIFT_INCREMENT) //|| 
//				DriverInputControl
//				.getInstance().getButton(RobotButtonType.TOTE_LIFT_NUDGE_UP)
				)
				&& !isIncrementing || isIncrementing) {

			if (!isIncrementing) {
				this.reset();

				liftIncrementHeight = 1210;

//				if (DriverInputControl.getInstance().getButton(
//						RobotButtonType.TOTE_LIFT_NUDGE_UP)) {
//					liftIncrementHeight = 250;
//				}
			}

			state = MotorState.UP;

			this.isIncrementing = !incrementLift(liftIncrementHeight, 10);

		}

		if ((DriverInputControl.getInstance().getButton(
				RobotButtonType.TOTE_LIFT_DECREMENT) //|| DriverInputControl
//				.getInstance().getButton(RobotButtonType.TOTE_LIFT_NUDGE_DOWN)
				)
				&& !isDecrementing || isDecrementing) {

			if (!isDecrementing) {
				this.reset();

				liftIncrementHeight = -1210;

//				if (DriverInputControl.getInstance().getButton(
//						RobotButtonType.TOTE_LIFT_NUDGE_DOWN)) {
//
//					liftIncrementHeight = -250;
//
//				}
			}

			state = MotorState.DOWN;
			this.isDecrementing = !incrementLift(liftIncrementHeight, 10);

		}

		if (DriverInputControl.getInstance().getButton(
				RobotButtonType.TOTE_LIFT_RESET)
				&& !isResetting || isResetting) {

			if (!isResetting) {
				isResetting = true;
				state = MotorState.UP;
				liftSpeed = DEFAULT_LIFT_SPEED;
			}

			if (!SensorInputControl.getInstance().isActive(
					SensorType.MAGNET_SENSOR)) {
				isResetting = false;
				state = MotorState.STOP;
				liftSpeed = 0;
			}
		}

//		System.out.println(toString());
		
		if (DriverInputControl.getInstance().getPressureButton(
				RobotButtonType.TOTE_LIFT) > DEAD_ZONE) {

			state = MotorState.DOWN;
			this.isIncrementing = false;
			this.isDecrementing = false;
			this.isResetting = false;
			this.reset();
			liftSpeed = DriverInputControl.getInstance().getPressureButton(
					RobotButtonType.TOTE_LIFT);
		} else if (DriverInputControl.getInstance().getPressureButton(
				RobotButtonType.TOTE_LIFT) < -DEAD_ZONE) {
			this.reset();
			this.isIncrementing = false;
			this.isDecrementing = false;
			this.isResetting = false;
			state = MotorState.UP;
			liftSpeed = DriverInputControl.getInstance().getPressureButton(
					RobotButtonType.TOTE_LIFT);

		} else if (!isIncrementing && !isDecrementing && !isResetting) {
			state = MotorState.STOP;
		}

		if (state == MotorState.STOP) {
			isBraked = true;
		} else {
			isBraked = false;
		}

		if (DriverInputControl.getInstance().getButton(
				RobotButtonType.HARD_STOP)) {
			isBraked = false;
		}

		if (state == MotorState.UP) {
			liftSpeed = (liftSpeed < DEFAULT_LIFT_SPEED ? DEFAULT_LIFT_SPEED
					: liftSpeed);
			if (SensorInputControl.getInstance()
					.getLimitSwitch(SensorType.TOTE_UPPER_LIMIT_SWITCH).get()) {
				stop();
			}
			updateLift(liftIncrementHeight);
		} else if (state == MotorState.DOWN) {
			liftSpeed = (liftSpeed > DEFAULT_LIFT_SPEED_DOWN ? DEFAULT_LIFT_SPEED_DOWN
					: liftSpeed);
			// liftSpeed = -DEFAULT_LIFT_SPEED;
			updateLift(liftIncrementHeight);
		} else {
			stop();
		}
	}

	public void updateLift(double speed) {
		liftSpeed = speed;
		if (speed < 0) {
			state = MotorState.UP;
		} else if (speed > 0) {
			state = MotorState.DOWN;
		} else {
			state = MotorState.STOP;
		}

		if (state == MotorState.UP) {
			liftSpeed = (liftSpeed < DEFAULT_LIFT_SPEED ? DEFAULT_LIFT_SPEED
					: liftSpeed);
			if (SensorInputControl.getInstance()
					.getLimitSwitch(SensorType.TOTE_UPPER_LIMIT_SWITCH).get()) {
				stop();
			}
		} else if (state == MotorState.DOWN) {
			liftSpeed = (liftSpeed > DEFAULT_LIFT_SPEED_DOWN ? DEFAULT_LIFT_SPEED_DOWN
					: liftSpeed);
			// liftSpeed = -DEFAULT_LIFT_SPEED;
		} else {
			stop();
		}
		
		if (state == MotorState.STOP) {
			isBraked = true;
		} else {
			isBraked = false;
		}

		if (state == MotorState.UP) {
			if (SensorInputControl.getInstance()
					.getLimitSwitch(SensorType.TOTE_UPPER_LIMIT_SWITCH).get()) {
				stop();
			}
		}
	}

	public boolean incrementLift(double distance, double error) {

        distanceTraveled = -SensorInputControl.getInstance()
                .getEncoder(SensorType.TOTE_ENCODER).getDistance();

        double difference = Math.abs(distanceTraveled - distance);
        boolean isDone = difference <= error;

        System.out
                .println("incrementLift - [encoder distance, wanted distance, error, difference, isDone]"
                        + distanceTraveled
                        + ", "
                        + distance
                        + ", "
                        + error
                        + ", " + difference + ", " + isDone);

        if (isDone) {
            this.reset();
            this.stop();
            return true;
        } else {
            liftSpeed = distanceControlLoop.getPID(distance, distanceTraveled);
            updateLift(-liftSpeed);
            System.out.println("incrementLift speed - " + liftSpeed);
            return false;
        }
    }
	
	public void incrementLift(double distance) {
		motorPower = -SensorInputControl.getInstance()
				.getEncoder(SensorType.TOTE_ENCODER).getRate();

		liftSpeed = motorPowerControlLoop.getPID(distance, motorPower);
		updateLift(-liftSpeed);
	}

	public void reset() {
		distanceControlLoop.reset();
		SensorInputControl.getInstance().getEncoder(SensorType.TOTE_ENCODER)
				.reset();
	}

	public void cycle() {
		if (state == MotorState.UP) {
			liftSpeed = DEFAULT_LIFT_SPEED;
			if (SensorInputControl.getInstance()
					.getLimitSwitch(SensorType.TOTE_LOWER_LIMIT_SWITCH).get()) {
				stop();
			}
		}
	}

	public void stop() {
		state = MotorState.STOP;
		liftSpeed = 0;
	}

	public void updateOutputs() {
		RobotControl.getInstance().updateToteMotor(-liftSpeed);
		RobotControl.getInstance().updateToteStop(isBraked);
	}

	@Override
	public String toString() {
		return "ToteLift [toteHeight=" + toteHeight + ", liftSpeed="
				+ liftSpeed + ", state=" + state + ", distanceControlLoop="
				+ distanceControlLoop + ", distanceTraveled="
				+ distanceTraveled + ", DEAD_ZONE=" + DEAD_ZONE
				+ ", isIncrementing=" + isIncrementing + ", isBraked="
				+ isBraked + "]";
	}

	@Override
	public void update() {
		// TODO Auto-generated method stub
		updateLift();
	}

}