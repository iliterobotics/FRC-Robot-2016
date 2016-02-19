package org.usfirst.frc.team1885.robot.common;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * an Improvement on the PID class, rewritten with better documentation and cleaner code
 * 
 * PID - A function which generates a sum of three functions in order to evaluate the error in a
 * system and produce the percentage of output that system should give. This function should be adaptive
 * to any system
 * 
 * @author Michael Kelly
 */
public class TruePID {

    /** the first time reading of the integral function*/
    private long initialTime;
    /** the previous time reading of the integral function*/
    private long lastTime;
    /** the total time of the integral function*/
    private long totalTime;
    
    /** the sum of all of the recorded errors weighted by the times that they occurred*/
    private double integral;
    /** represents whether or not this is the first time that the derivative function was called */
    private boolean firstD;
    /** the last error value that was recorded*/
    private double lastError;
    
    /**The base proportion of the error to the output. Alias: kp*/
    private final double P;
    /**The proportion of the integral of the error/time function to the output. Alias: ki*/
    private final double I;
    /**The proportion of the derivative function to the output. Alias: kd*/
    private final double D;
    /**The amount of error that achieves a maximum output*/
    private final double MAX_ERROR;
    
    
    /**
     * @see TruePID
     * @param P the base proportion in the function ranging from 0 to 1
     * @param I the integral proportion in the function ranging from 0 to 1
     * @param D the derivative proportion in the function ranging from 0 to 1
     * @param MAX_ERROR the amount of error which always yields a 100% output;
     */
    public TruePID(final double P, final double I, final double D, final double MAX_ERROR){
        this.P = P;
        this.I = I;
        this.D = D;
        this.MAX_ERROR = MAX_ERROR;
        
        initialTime = -1;
        firstD = true;
    }
    
    /**
     * Calculates the sum of the proportion, integral, and derivative functions in
     * order to return an output ranging from -1 to 1
     * @param projectedValue the value which the system is trying to achieve
     * @param currentValue the value which the system is currently at
     * @return the sum of {@link #getP(error)}, {@link #getI(error)}, and {@link #getD(error)} at a maximum of 100%
     * and a minimum of -100%
     */
    public double getPID(double projectedValue, double currentValue){
        
        double error = (projectedValue - currentValue) / MAX_ERROR;
        
        DriverStation.reportError("\nerror:" + error, false);
        DriverStation.reportError("\nP:" + getP(error), false);
        DriverStation.reportError("\nI:" + getI(error, 1000), false);
        DriverStation.reportError("\nD:" + getD(error), false);
        
        double pid = getP(error) + getI(error, 1000) + getD(error);
        
        return pid;
    }
    
    /**
     * <p>Uses a direct proportion to generate an output</p>
     * 
     * P = kp * %error
     * 
     * @param error the percentage of error ranging from -1 to 1
     * @return the amount of power to be applied to the system ranging from -1 to 1     
     */
    public double getP(double error){
        return P * error;
    }
    
    /**
     * <p>
     * Uses the integral of the error over time graph to calculate how much
     * the output should be increased based on the summation of all of the 
     * errors over time
     * </p>
     * 
     * I = ki * ((error1*time1 + error2*time2 ... errorn*timen)/totalTime)
     * 
     * @param error the percentage of error ranging from -1 to 1
     * @param timeDelimeter the number of milliseconds to yield full error
     * @return the amount of power to be applied to the system ranging from -1 to 1
     */
    public double getI(double error, int timeDelimeter){
        integral  += error;
        
        double result =  I * (integral);
        
        return result;
    }
    
    /**
     * <p>
     * Calculates the current value of the derivative of the error time graph
     * scaled by the specified D proportion 
     * </p>
     * D = kd * (error - previous error)
     * @param error the percentage of error ranging from -1 to 1
     * @return the amount of power to be applied to the system ranging from -1 to 1
     */
    public double getD(double error){
        //check to see if this is the first time we check the derivative
        if(firstD){
            lastError = error;
            firstD = false;
        }
        double result = D * (error - lastError);
        lastError = error;
        return result;
    }
    
    /**
     * resets the PID
     */
    public void reset(){
        initialTime = -1;
        firstD = true;
        
        integral = 0;
        totalTime= 0;
    }
}
