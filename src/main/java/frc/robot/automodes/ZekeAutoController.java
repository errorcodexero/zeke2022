package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.AutoMode;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import frc.robot.Zeke2022;

//
// This class is the automode controller for Zeke.  It basically creates all of the
// automodes and picks one based on test mode and the automode controller selected.
//
public class ZekeAutoController extends AutoController {
    private AutoMode test_mode_ ;
    private AutoMode [] modes_ ;

    public ZekeAutoController(Zeke2022 robot) throws MissingParameterException, BadParameterTypeException {
        super(robot, "zeke-auto");

        MessageLogger logger = getRobot().getMessageLogger() ;
        modes_ = new AutoMode[10] ;
        
        try {
            test_mode_ = new ZekeTestModeAuto(this);
            modes_[0] = new TwoBallLeftAuto(this, "Two Left");
            modes_[1] = new TwoBallRightAuto(this, "Two Right");
            modes_[2] = new ThreeBallAuto(this, "Three Ball");
            modes_[3] = new FourBallAuto(this, "Four Ball");
            modes_[4] = new PitTestAutoMode(this, "Pit Test") ;
            modes_[5] = new NoOpAuto(this, "Nop-5") ;
            modes_[6] = new NoOpAuto(this, "Nop-6") ;
            modes_[7] = new NoOpAuto(this, "Nop-7") ;
            modes_[8] = new NoOpAuto(this, "Nop-8") ;
            modes_[9] = new NoOpAuto(this, "Nop-9") ;                                    
        }
        catch(Exception e) {
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create automode 'ZekeTestModeAuto', exception caught - ") ;
            logger.add(e.getMessage()) ;
            logger.endMessage();
        }
    }

    public void updateAutoMode(int mode, String gamedata) throws Exception {
        if (isTestMode()) {
            setAutoMode(test_mode_) ;
        }
        else {
            if (mode >= 0 && mode < modes_.length) {
                if (getAutoMode() != modes_[mode])
                    setAutoMode(modes_[mode]) ;
            }
        }
    }
}
