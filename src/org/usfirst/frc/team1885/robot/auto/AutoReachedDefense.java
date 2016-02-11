package org.usfirst.frc.team1885.robot.auto;

public class AutoReachedDefense extends AutoCommand{
    //autonomous checkpoint used to check anything before we reach a defense
    
    
    
    @Override
    public boolean init() {
        return true;
    }

    @Override
    public boolean execute() {
//        if(){
//           return true; 
//        }
        return false;
    }

    @Override
    public boolean updateOutputs() {
        // no outputs to update, only a checkpoint
        return false;
    }

    @Override
    public void reset() {
        // no outputs changed, only a checkpoint
    }

}
