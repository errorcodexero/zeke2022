package frc.robot.climber;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.base.pneumatics.XeroDoubleSolenoid;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ClimberSubsystem extends Subsystem {
    
    public static final String SubsystemName = "climber" ;

    // 2 "windmills" which spin like windmills. 
    // 1 on left side of robot and 1 on right side
    // they follow each other so only one motorencoder subsystem is defined here
    private MotorEncoderSubsystem windmill_ ;

    // 2 double-solenoids
    // connect between the two windmills; 1 on either end
    private XeroDoubleSolenoid clamp_a_ ;
    private XeroDoubleSolenoid clamp_b_ ;

    private static DoubleSolenoid.Value GripperCloseState = DoubleSolenoid.Value.kForward ;
    private static DoubleSolenoid.Value GripperOpenState = DoubleSolenoid.Value.kReverse ;

    // there are 6 touch-sensors; aka wobble switch sensors
    // there are 2 per each "a-clamp" and 1 per each "b-clamp"
    // they are named after which bar they'll hit/sensee (medium, high, traversal)
    // they are also named after which side of the robot they're on (left or right)
    private DigitalInput mid_left_ ;
    private DigitalInput mid_right_ ;
    private DigitalInput high_left_ ;
    private DigitalInput high_right_ ;
    private DigitalInput traversal_left_ ;
    private DigitalInput traversal_right_ ;
    
    // perhaps rename these as "A to B" and "B to A"
    private MotorEncoderPowerAction windmill_power_forwards_; 
    private MotorEncoderPowerAction windmill_power_backwards_; 
    private MotorEncoderPowerAction windmill_power_off_; 

    public static enum ChangeClampTo {
        OPEN, 
        CLOSED, 
        UNKNOWN
    }

    public static enum SetWindmillTo {
        FORWARDS, 
        BACKWARDS, 
        OFF,
        UNKNOWN
    }

    public static enum WhichClamp {
        CLAMP_A,
        CLAMP_B
    }

    public ClimberSubsystem(Subsystem parent) throws Exception {
        super(parent, SubsystemName);

        windmill_ = new MotorEncoderSubsystem(parent, SubsystemName, true) ;

        clamp_a_ = new XeroDoubleSolenoid(this, "clamp_a") ;
        clamp_b_ = new XeroDoubleSolenoid(this, "clamp_b") ;

        double doublyindex ;
        doublyindex = getSettingsValue("hw:windmill:windmill_power_forwards").getDouble() ;
        windmill_power_forwards_ = new MotorEncoderPowerAction(windmill_, doublyindex) ;
        doublyindex = getSettingsValue("hw:windmill:windmill_power_backwards").getDouble() ;
        windmill_power_backwards_ = new MotorEncoderPowerAction(windmill_, doublyindex) ;
        doublyindex = getSettingsValue("hw:windmill:windmill_power_off").getDouble() ;
        windmill_power_off_ = new MotorEncoderPowerAction(windmill_, doublyindex) ;

        int index ;
        index = getSettingsValue("hw:touchsensors:mid_left").getInteger() ;
        mid_left_ = new DigitalInput(index) ;
        index = getSettingsValue("hw:touchsensors:mid_right").getInteger() ;
        mid_right_ = new DigitalInput(index) ;
        index = getSettingsValue("hw:touchsensors:high_left").getInteger() ;
        high_left_ = new DigitalInput(index) ;
        index = getSettingsValue("hw:touchsensors:high_right").getInteger() ;
        high_right_ = new DigitalInput(index) ;
        index = getSettingsValue("hw:touchsensors:traversal_left").getInteger() ;
        traversal_left_ = new DigitalInput(index) ;
        index = getSettingsValue("hw:touchsensors:traversal_right").getInteger() ;
        traversal_right_ = new DigitalInput(index) ;
    }

    public MotorEncoderSubsystem getWindmillMotor() {
        return windmill_ ;
    }

    //windmills
    public void setWindmill(SetWindmillTo windmill_power_) throws BadMotorRequestException, MotorRequestFailedException {
        switch (windmill_power_) {
            case OFF:
                windmill_.setAction(windmill_power_off_);
                break ;
            case FORWARDS:
                windmill_.setAction(windmill_power_forwards_);
                break ;
            case BACKWARDS:
                windmill_.setAction(windmill_power_backwards_) ;
                break ;
            default:
                windmill_.setAction(windmill_power_off_);
                break ;
        }
    }

    //clamps. 
    // ensure both l and r are clamping/unclamping simultaneously given they're on the same end of the windmill
    // also, set the clamp to neither open nor closed if passed in parameter is "UNKNWOWN" 
    public void changeClamp(WhichClamp clamp_name, ChangeClampTo clamp_setting) {
        MessageLogger logger = getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getLoggerID()) ;
        logger.add("Climber: set clamp: ") ;
        // logger.add(changeClamp.toString()) ;
        logger.endMessage();
        
        if (clamp_name == WhichClamp.CLAMP_A) {
            if (clamp_setting == ChangeClampTo.CLOSED) {
                clamp_a_.set(GripperCloseState);
            }
            else if (clamp_setting == ChangeClampTo.OPEN) {
                clamp_a_.set(GripperOpenState);
            }
        } else if (clamp_name == WhichClamp.CLAMP_B) {
            if (clamp_setting == ChangeClampTo.CLOSED) {
                clamp_b_.set(GripperCloseState);
            }
            else if (clamp_setting == ChangeClampTo.OPEN) {
                clamp_b_.set(GripperOpenState);
            }
        }
    }
 
    //touch sensors
    public boolean isMidLeftTouched() {
        return mid_left_.get() ;
    }
    public boolean isMidRightTouched() {    
        return mid_right_.get() ;
    }
    public boolean isHighLeftTouched() {
        return high_left_.get();
    }
    public boolean isHighRightTouched() {
        return high_right_.get();
    }
    public boolean isTraversalLeftTouched() {
        return traversal_left_.get();
    }
    public boolean isTraversalRightTouched() {
        return traversal_right_.get();
    }

}
