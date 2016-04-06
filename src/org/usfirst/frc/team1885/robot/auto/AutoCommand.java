package org.usfirst.frc.team1885.robot.auto;

/**
 * @author ILITE Robotics
 * @version<2/13/2016>
 */
public abstract class AutoCommand {
    
    protected double TIMEOUT = 5000;
    protected boolean timeSet = false;
    protected boolean isInit = false;
    protected long initTime;

    /**
     * @return True.
     */
    public abstract boolean init();

    /**
     * Method that executes bulk of code. Primarily used in conjunction with the
     * moving of the Drive Train and Manipulators
     * 
     * @return True when method determines processes to be complete. False
     *         otherwise.
     */
    public abstract boolean execute();

    /**
     * Updates the global informative variables. These variables directly
     * correlate to the actions of the Drive Train.
     * 
     * @return False.
     */
    public abstract boolean updateOutputs();

    /**
     * Resets any potential values that may need to be reset.
     */
    public abstract void reset();

    public boolean isInit() {
        return isInit;
    }
    public void setInit(boolean isInit) {
        this.isInit = isInit;
    }
    public boolean timeOut(){
        if(!timeSet){
            initTime = System.currentTimeMillis();
            timeSet = true;
        }
        return initTime - System.currentTimeMillis() > TIMEOUT;
    }
}
