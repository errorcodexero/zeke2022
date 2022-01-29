package frc.robot.climber;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.pneumatics.XeroDoubleSolenoid;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ClimberSubsystem extends Subsystem {
    
    public static final String SubsystemName = "climber" ;

    // two "windmills" whcih spin like windmills. 
    // one on left side of robot and one on right side 
    private MotorController left_windmill_ ;
    private MotorController right_windmill_ ;

    // 4 double-solenoids
    // 2 on each windmill; 1 on either end
    private XeroDoubleSolenoid clamp_a_left_ ;
    private XeroDoubleSolenoid clamp_b_left_ ;
    private XeroDoubleSolenoid clamp_a_right_ ;
    private XeroDoubleSolenoid clamp_b_right_ ;

    // there are 6 touch-sensors; aka wobble switch sensors
    // there are 2 per each "a-clamp" and 1 per each "b-clamp"
    // they are named after which bar they'll hit (medium, high, traversal)
    // they are also named after which side of the robot they're on (left or right)
    private DigitalInput mid_left_ ;
    private DigitalInput mid_right_ ;
    private DigitalInput high_left_ ;
    private DigitalInput high_right_ ;
    private DigitalInput traversal_left_ ;
    private DigitalInput traversal_right_ ;

    public ClimberSubsystem(Subsystem parent) throws Exception {
        super(parent, SubsystemName);
        
        left_windmill_ = getRobot().getMotorFactory().createMotor(SubsystemName, "subsystems:climber:hw:motors:left_windmill") ;
        right_windmill_ = getRobot().getMotorFactory().createMotor(SubsystemName, "subsystems:climber:hw:motors:right_windmill") ;

        // TODO: make the integers into params
        clamp_a_left_ = new XeroDoubleSolenoid(getRobot(), 1, -1) ;
        clamp_b_left_ = new XeroDoubleSolenoid(getRobot(), 1, -1) ;
        clamp_a_right_ = new XeroDoubleSolenoid(getRobot(), 1, -1) ;
        clamp_b_right_ = new XeroDoubleSolenoid(getRobot(), 1, -1) ;

        // TODO: make the channels into params
        mid_left_ = new DigitalInput(0) ;
        mid_right_ = new DigitalInput(1) ;
        high_left_ = new DigitalInput(2) ;
        high_right_ = new DigitalInput(3) ;
        traversal_left_ = new DigitalInput(4) ;
        traversal_right_ = new DigitalInput(5) ;
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
    public void setLeftWindmill(double percent) throws BadMotorRequestException, MotorRequestFailedException {
        left_windmill_.set(percent);
    }
    public void setRightWindmill(double percent) throws BadMotorRequestException, MotorRequestFailedException {
        right_windmill_.set(percent);
    }

    //clamps
    public void setClampALeft(DoubleSolenoid.Value state_) {
        clamp_a_left_.set(state_);
    }
    public void setClampARight(DoubleSolenoid.Value state_) {
        clamp_a_right_.set(state_);
    }
    public void setClampBLeft(DoubleSolenoid.Value state_) {
        clamp_b_left_.set(state_);
    }
    public void setClampBRight(DoubleSolenoid.Value state_) {
        clamp_b_right_.set(state_);
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
