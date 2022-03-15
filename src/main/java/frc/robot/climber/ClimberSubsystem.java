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
    // two clamps on a, and 2 on b - left and right
    // they are also named after which side of the robot they're on (left or right)
    // either A's or B's on the bars at one point in time
    private DigitalInput left_a_ ;
    private DigitalInput right_a_ ;
    private DigitalInput left_right_b_ ;
    
    // perhaps rename these as "A to B" and "B to A"
    private MotorEncoderPowerAction windmill_power_forwards_; 
    private MotorEncoderPowerAction windmill_power_backwards_; 
    private MotorEncoderPowerAction windmill_power_off_; 

    private GrabberState a_grabbers_ ;
    private GrabberState b_grabbers_ ;

    private boolean left_a_switch_ ;
    private double left_a_start_ ;
    private boolean right_a_switch_ ;
    private double right_a_start_ ;
    private boolean left_b_switch_ ;
    private double left_b_start_ ;
    private boolean right_b_switch_ ;
    private double right_b_start_ ;
    private double now_ ;

    public static enum GrabberState {
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
        addChild(windmill_) ;

        // Reset encoders so that the startup position is the zero position
        windmill_.reset() ;

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
        index = getSettingsValue("hw:touchsensors:left_a").getInteger() ;
        left_a_ = new DigitalInput(index) ;
        index = getSettingsValue("hw:touchsensors:right_a").getInteger() ;
        right_a_ = new DigitalInput(index) ;
        index = getSettingsValue("hw:touchsensors:left_right_b").getInteger() ;
        left_right_b_ = new DigitalInput(index) ;

        a_grabbers_ = GrabberState.UNKNOWN ;
        b_grabbers_ = GrabberState.UNKNOWN ;
    }

    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();

        now_ = getRobot().getTime() ;

        boolean v ;

        v = left_a_.get() ;
        if (v && !left_a_switch_)
            left_a_start_ = now_ ;
        left_a_switch_ = v ;

        v = right_a_.get() ;
        if (v && !right_a_switch_)
            right_a_start_ = now_ ;
        right_a_switch_ = v ;

        v = left_right_b_.get() ;
        if (v && !left_b_switch_)
            left_b_start_ = now_ ;
        left_b_switch_ = v ;

        v = left_right_b_.get() ;
        if (v && !right_b_switch_)
            right_b_start_ = now_ ;
        right_b_switch_ = v ;

        putDashboard("climber", DisplayType.Always, windmill_.getPosition());
    }

    public MotorEncoderSubsystem getWindmillMotor() {
        return windmill_ ;
    }

    //windmills
    // public void setWindmill(SetWindmillTo windmill_power_) throws BadMotorRequestException, MotorRequestFailedException {
    //     switch (windmill_power_) {
    //         case OFF:
    //             windmill_.setAction(windmill_power_off_);
    //             break ;
    //         case FORWARDS:
    //             windmill_.setAction(windmill_power_forwards_);
    //             break ;
    //         case BACKWARDS:
    //             windmill_.setAction(windmill_power_backwards_) ;
    //             break ;
    //         default:
    //             windmill_.setAction(windmill_power_off_);
    //             break ;
    //     }
    // }

    public GrabberState getClampState(WhichClamp clamp_name) {
        return (clamp_name == WhichClamp.CLAMP_A) ? a_grabbers_ : b_grabbers_ ;
    }

    //clamps. 
    // ensure both l and r are clamping/unclamping simultaneously given they're on the same end of the windmill
    // also, set the clamp to neither open nor closed if passed in parameter is "UNKNWOWN" 
    public void changeClamp(WhichClamp clamp_name, GrabberState clamp_setting) {
        MessageLogger logger = getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getLoggerID()) ;
        logger.add("Climber: set clamp: ") ;
        logger.add(clamp_name.toString()) ;
        logger.add(" to ") ;
        logger.add(clamp_setting.toString()) ;
        logger.endMessage();
        
        if (clamp_name == WhichClamp.CLAMP_A) {
            if (clamp_setting == GrabberState.CLOSED) {
                clamp_a_.set(GripperCloseState);
            }
            else if (clamp_setting == GrabberState.OPEN) {
                clamp_a_.set(GripperOpenState);
            }
            a_grabbers_ = clamp_setting ;

        } else if (clamp_name == WhichClamp.CLAMP_B) {
            if (clamp_setting == GrabberState.CLOSED) {
                clamp_b_.set(GripperCloseState);
            }
            else if (clamp_setting == GrabberState.OPEN) {
                clamp_b_.set(GripperOpenState);
            }

            b_grabbers_ = clamp_setting ;
        }
    }
 
    //touch sensors
    public boolean isLeftATouched() {
        return left_a_switch_ ;
    }
    public boolean isRightATouched() {    
        return right_a_switch_ ;
    }
    public boolean isLeftBTouched() {
        return left_b_switch_ ;
    }
    public boolean isRightBTouched() {
        return right_b_switch_ ;
    }

    public double leftADuration() {
        return now_ - left_a_start_ ;
    }

    public double rightADuration() {
        return now_ - right_a_start_ ;
    }

    public double leftBDuration() {
        return now_ - left_b_start_ ;
    }

    public double rightBDuration() {
        return now_ - right_b_start_ ;
    }
}
