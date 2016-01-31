package org.usfirst.frc.team1885.robot.auto;

public class AutoCheckSelf extends AutoCommand {
    //true is facing the defenses, false is facing parallel to the defenses
    boolean isStraight;
    
    public AutoCheckSelf(){
        init();
    }
    
    public AutoCheckSelf(boolean b){
        isStraight = b;
        init();
    }
    
    @Override
    public boolean init() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean execute() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean updateOutputs() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        
    }

}
