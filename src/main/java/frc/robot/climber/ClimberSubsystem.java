package frc.robot.climber;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.base.pneumatics.XeroDoubleSolenoid;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ClimberSubsystem extends Subsystem {
    
    public static final String SubsystemName = "climber" ;
    
    // Butch: both of these motors should 100% be controlled together with one
    //        being a follower of the other.  THis means you only need a single subsystem
    //        and it will control both.  To get the simulator running I removed the right_windmill_
    //        MotorEncoderSubsystem and used the left one as the single subsystem.

    // 2 "windmills" whcih spin like windmills. 
    // 1 on left side of robot and 1 on right side 
    private MotorEncoderSubsystem left_windmill_ ;
    // private MotorEncoderSubsystem right_windmill_ ;

    // 4 double-solenoids
    // 2 on each windmill; 1 on either end
    private XeroDoubleSolenoid clamp_a_left_ ;
    private XeroDoubleSolenoid clamp_b_left_ ;
    private XeroDoubleSolenoid clamp_a_right_ ;
    private XeroDoubleSolenoid clamp_b_right_ ;

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

    public static enum ChangeClampTo {
        OPEN, 
        CLOSED, 
        UNKNOWN
    }
    private ChangeClampTo changeClampA = ChangeClampTo.UNKNOWN ;
    private ChangeClampTo changeClampB = ChangeClampTo.UNKNOWN ;

    public ClimberSubsystem(Subsystem parent) throws Exception {
        super(parent, SubsystemName);
        int index ;

        left_windmill_ = new MotorEncoderSubsystem(parent, SubsystemName, true) ;
        // right_windmill_ = new MotorEncoderSubsystem(parent, SubsystemName, true) ;

        clamp_a_left_ = new XeroDoubleSolenoid(this, "clamp_a_left") ;
        clamp_b_left_ = new XeroDoubleSolenoid(this, "clamp_b_left") ;
        clamp_a_right_ = new XeroDoubleSolenoid(this, "clamp_a_right") ;
        clamp_b_right_ = new XeroDoubleSolenoid(this, "clamp_b_right") ;
       
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

    @Override
    public void run() throws Exception {
        super.run() ;
    }

    @Override
    public void postHWInit() {
        // setDefaultAction(new ClimberStopAction(this));
    }

    @Override
    public void computeMyState() throws Exception {

    }

    //windmills
    // TODO get this percent from params file
    public void setWindmill(double power) throws BadMotorRequestException, MotorRequestFailedException {
        left_windmill_.setPower(power); 
        // right_windmill_.setPower(power); 
    }

    //clamps. 
    // ensure both l and r are clamping/unclamping simultaneously given they're on the same end of the windmill
    // also, set the clamp to neither open nor closed if passed in parameter is "UNKNWOWN" 
    public void setClampAClosed(ChangeClampTo ChangeClampA) {
        if (ChangeClampA == ChangeClampTo.CLOSED) {
            clamp_a_left_.set(GripperCloseState);
            clamp_a_right_.set(GripperCloseState);
        } else if (ChangeClampA == ChangeClampTo.OPEN) {
            clamp_a_left_.set(GripperOpenState);
            clamp_a_right_.set(GripperCloseState);
        } 
    }
    public void setClampBClosed(ChangeClampTo ChangeClampB) {
        if (ChangeClampB == ChangeClampTo.CLOSED) {
            clamp_a_left_.set(GripperCloseState);
            clamp_a_right_.set(GripperCloseState);
        } else if (ChangeClampB == ChangeClampTo.OPEN) {
            clamp_a_left_.set(GripperOpenState);
            clamp_a_right_.set(GripperCloseState);
        }
    }
 
    //touch sensors
    public boolean isMidLeftTouched() {
        if (mid_left_.get() == true)
            return true ;
        else 
            return false;
    }
    public boolean isMidRightTouched() {
        if (mid_right_.get() == true)
            return true ;
        else 
            return false;
    }
    public boolean isHighLeftTouched() {
        if (high_left_.get() == true)
            return true ;
        else 
            return false;
    }
    public boolean isHighRightTouched() {
        if (high_right_.get() == true)
            return true ;
        else 
            return false;
    }
    public boolean isTraversalLeftTouched() {
        if (traversal_left_.get() == true)
            return true ;
        else 
            return false;
    }
    public boolean isTraversalRightTouched() {
        if (traversal_right_.get() == true)
            return true ;
        else 
            return false;
    }

}
