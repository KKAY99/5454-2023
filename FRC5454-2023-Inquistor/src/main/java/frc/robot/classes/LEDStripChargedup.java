package frc.robot.classes;

public class LEDStripChargedup extends LEDStrip{

    public static enum LEDMode
    {
                    NOTSET, DISABLED, AUTOMODE, CUBE,CONE,BALANCED;	
    }
    private LEDMode m_oldLEDMode;
    private LEDMode m_LEDMode;
    public LEDStripChargedup(int port, int leds){
        super (port,leds);
        m_oldLEDMode=LEDMode.NOTSET;
    }
    public void updateLED(){
        if(m_oldLEDMode!=m_LEDMode){
            m_oldLEDMode=m_LEDMode;
            super.update();
    
        }
        
    }

public void setRobotMode(LEDMode ledMode){
    switch(ledMode){
        case NOTSET:
        break;
        case DISABLED:
        break;
        case AUTOMODE:
        break;
        case CUBE:
        break;
        case CONE:
        break;
        case BALANCED:
        break;
    }
    updateLED();
}

}
