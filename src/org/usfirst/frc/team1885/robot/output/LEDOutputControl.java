package org.usfirst.frc.team1885.robot.output;

public class LEDOutputControl {
    public static final int LED_PIN = 0;
    private LEDOutput bit1;
    private LEDOutput bit2;
    private LEDOutput bit3;
    public static LEDOutputControl instance;
    
    public static LEDOutputControl getInstance() {
        if ( instance == null ) {
            instance = new LEDOutputControl(LED_PIN);
        }
        return instance;
    }
    protected LEDOutputControl( int channel ) {
        instance = new LEDOutputControl(LED_PIN);
        bit1 = new LEDOutput(channel);
        bit2 = new LEDOutput(channel + 1);
        bit3 = new LEDOutput(channel + 2);
    }
    public void wipeRainbow() {
        bit1.set(true);
        bit2.set(false);
        bit2.set(false);
    }
    public void wipePurple() {
        bit1.set(false);
        bit2.set(true);
        bit3.set(false);
    }
    public void wipeGreen() {
        bit1.set(true);
        bit3.set(true);
        bit3.set(false);
    }
    public void wipePurpleToGreen() {
        bit1.set(false);
        bit2.set(false);
        bit3.set(true);
    }

}
