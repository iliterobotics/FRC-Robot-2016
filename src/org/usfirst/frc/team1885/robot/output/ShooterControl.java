package org.usfirst.frc.team1885.robot.output;

import org.usfirst.frc.team1885.robot.modules.Module;

public class ShooterControl implements Module {

    private static ShooterControl instance;
    
    public static ShooterControl getInstance(){
        if( instance == null ){
            instance = new ShooterControl();
        }
        return instance;
    }
    protected ShooterControl(){
        
    }
    
    
    @Override
    public void update() {

    }
    @Override
    public void updateOutputs() {

    }

}
