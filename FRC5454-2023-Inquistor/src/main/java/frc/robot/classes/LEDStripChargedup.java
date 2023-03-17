package frc.robot.classes;
import frc.robot.Constants.LEDS.Colors;
import frc.robot.Constants;
public class LEDStripChargedup extends LEDStrip{
    private static final int LEDMODE_WAVE = 0;
    private static final int LEDMODE_BAR = 1;
    private static final int LEDMODE_RAINBOW = 2;
    private static final int LEDMODE_SOLID = 3;
    private static final int LEDMODE_OFF = 4;
    private static boolean m_ledFlash=false;    
    private static boolean m_ledFlashState=false;
    public static enum LEDMode
    {
                    NOTSET, DISABLED, AUTOMODE,TELEOP, CUBE,CONE,BALANCED;	
    }
    private LEDMode m_oldLEDMode;
    private LEDMode m_LEDMode;
    private int m_ledFlashDelayCount=0;
    public LEDStripChargedup(int port, int leds){
        super (port,leds);
        m_oldLEDMode=LEDMode.NOTSET;
    }
    
    public void updateLED(){
        //only update if led mode is changing or in flash mode
        if(m_oldLEDMode!=m_LEDMode){
            m_oldLEDMode=m_LEDMode;
            super.update();    
        } else {
            if (m_ledFlash){
                flashmode();
                super.update();
            }
        }
        
    }

public void setRobotMode(LEDMode ledMode){
    switch(ledMode){
        case NOTSET:
        break;
        case DISABLED:
            super.setMode(LEDMODE_WAVE);
            super.setColor(Colors.PURPLE);
            m_ledFlash=false;
        break;
        case AUTOMODE:
            super.setColor(Colors.PURPLE);
            super.setMode(LEDMODE_RAINBOW);
            m_ledFlash=false;
        break;
        case TELEOP:
            super.setMode(LEDMODE_WAVE);
            super.setColor(Colors.GREEN);
             m_ledFlash=false;

        break;
        case CUBE:
        super.setMode(LEDMODE_SOLID);
        super.setColor(Colors.PURPLE);
        m_ledFlash=false;

        break;
        case CONE:
        super.setMode(LEDMODE_SOLID);
        super.setColor(Colors.YELLOW);
        m_ledFlash=false;

        break;
        case BALANCED:
        super.setMode(LEDMODE_SOLID);
        super.setColor(Colors.PURPLE);
        m_ledFlash=true;
    
        break;
    }
   updateLED();
    
}
public void flashmode(){
    
        m_ledFlashDelayCount++;
        if(m_ledFlashState==false){
                //(m_ledFlashDelayCouunt>10)
                if(m_ledFlashDelayCount>Constants.LEDS.FLASH_DELAY) {
                        super.setMode(LEDMODE_OFF);
                        m_ledFlashState=true;
                        m_ledFlashDelayCount=0;
                }
        } else{
                if(m_ledFlashDelayCount>Constants.LEDS.FLASH_DELAY) {                       
                        super.setMode(LEDMODE_SOLID);                        
                        m_ledFlashState=false;
                        m_ledFlashDelayCount=0;
                }
        }

}

}   
