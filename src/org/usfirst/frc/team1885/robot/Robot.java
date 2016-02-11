package org.usfirst.frc.team1885.robot;

import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.config2015.RobotConfigSRX;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends SampleRobot {
    private RobotControlWithSRX srx;
    private DriverInputControlSRX drx;
    private SensorInputControlSRX sensorrx;
    private AHRS navx;

    public Robot() {
        navx = new AHRS(SerialPort.Port.kMXP);
        System.err.println("I am ROBOT!!");
        RobotConfigSRX.configureRobot();
        srx = RobotControlWithSRX.getInstance();
        drx = DriverInputControlSRX.getInstance();
        this.sensorrx = SensorInputControlSRX.getInstance();
    }
    public void operatorControl() {
        System.err.println("HAS OPERATOR CONTROL");
        // PressureSensor ps = new PressureSensor(0);
        while (isOperatorControl() && isEnabled()) {
            StringBuilder output = new StringBuilder();
            // output.append("\n" + ps.getPressure());
            // drx.update();
            // sensorrx.update();

            // can't update faster than this for motors
            // output.append("test");
            // output.append( "\nIMU_Connected" + pengisbad.isConnected());
            // output.append( "\nIMU_IsCalibrating" +
            // pengisbad.isCalibrating());

//            output.append("\nIMU_Yaw" + navx.getYaw());
//            output.append("\nIMU_Pitch" + navx.getPitch());
//            output.append("\nIMU_Roll" + navx.getRoll());
//
//            output.append("\nRawGyro_X" + navx.getRawGyroX());
//            output.append("\nRawGyro_Y" + navx.getRawGyroY());
//            output.append("\nRawGyro_Z" + navx.getRawGyroZ());
            
//              output.append("\nEncoder Position: " + sensorrx.getEncoderPos( SensorType.DRIVE_TRAIN_ENCODER ) );
              output.append("\nEncoder Velocity: " + sensorrx.getEncoderVelocity( SensorType.DRIVE_TRAIN_ENCODER ) );
            /*
             * output.append( "\nRawAccel_X"+ pengisbad.getRawAccelX());
             * output.append( "\nRawAccel_Y"+ pengisbad.getRawAccelY());
             * output.append( "\nRawAccel_Z" + pengisbad.getRawAccelZ());
             * output.append( "\nRawMag_X" + pengisbad.getRawMagX());
             * output.append( "\nRawMag_Y" + pengisbad.getRawMagY());
             * output.append( "\nRaw2Mag_Z" + pengisbad.getRawMagZ());
             * output.append( "\nIMU_Temp_C" + pengisbad.getTempC());
             * 
             * output.append( "\nIMU_Accel_X" +
             * pengisbad.getWorldLinearAccelX()); output.append( "\nIMU_Accel_Y"
             * + pengisbad.getWorldLinearAccelY()); output.append(
             * "\nIMU_IsMoving" + pengisbad.isMoving()); output.append(
             * "\nIMU_IsRotating" + pengisbad.isRotating());
             * 
             */
            DriverStation.reportError(output.toString(), false);
            Timer.delay(1);
        }

    }
    // public void autonomous()
    // {
    // AutoRoutine ar = new AutoRoutine(this);
    // ar.execute();
    // }
}
