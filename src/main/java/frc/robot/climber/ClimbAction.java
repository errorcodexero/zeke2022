package frc.robot.climber;

import org.xero1425.base.actions.Action;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.tankdrive.TankDriveSubsystem;

public class ClimbAction extends Action {

    private ClimberSubsystem sub_ ;
    private TankDriveSubsystem db_ ;

    // powers to windmills
    private double first_windmill_power_ ;
    private double second_windmill_power_ ;

    private double state_start_time_ ;

    // delay times between clamping/unclamping sections
    // first/second = whether it's between mid-high or high-traversal
    // clamp = for the clamping to finish before windmill starts going around
    // align = for hooking/aligning the next clamper on the "opposite" end of the windmill
    // swing = for the robot to swing a bit after un-clamping so the next clamp doesn't have too much tension in it
    private double first_clamp_wait_ ;
    private double first_align_wait_ ;
    private double first_swing_wait_ ;
    private double second_clamp_wait_ ;
    private double second_align_wait_ ;
    private double second_swing_wait_ ;

    
    private enum ClimbingStates {
        IDLE,
        SQUARING,
        CLAMP_ONE,
        WINDMILL_ONE,
        UNCLAMP_ONE,
        CLAMP_TWO,
        WINDMILL_TWO,
        UNCLAMP_TWO,
        CLAMP_THREE
        //do we need DONE?
    }

    private ClimbingStates state_ = ClimbingStates.IDLE ;

    // TODO: grab the times/waits and windmill powers from params file
    public ClimbAction(ClimberSubsystem sub, TankDriveSubsystem db, 
                        double firstWindmillPower, double secondWindmillPower, 
                        double firstClampWait, double firstAlignWait, double firstSwingWait,
                        double secondClampWait, double secondAlignWait, double secondSwingWait) {
        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;
        db_ = db ;

        first_windmill_power_ = firstWindmillPower ;
        second_windmill_power_ = secondWindmillPower ;

        first_clamp_wait_ = firstClampWait ;
        first_align_wait_ = firstAlignWait ;
        first_swing_wait_ = firstSwingWait ;
        second_clamp_wait_ = secondClampWait ;
        second_align_wait_ = secondAlignWait ;
        second_swing_wait_ = secondSwingWait ;

    }

    @Override
    public void start() throws Exception {
        super.start() ;

        setDone() ;
    }


    @Override
    public void run() throws BadMotorRequestException, MotorRequestFailedException {
        switch (state_) {
            case IDLE:
                doIdle() ;
                break ;
            case SQUARING:
                doSquaring() ;
                break ;
            case CLAMP_ONE:
                doClampOne() ;
                break ;
            case WINDMILL_ONE:
                doWindmillOne() ;
                break ;
            case UNCLAMP_ONE:
                doUnclampOne() ;
                break ;
            case CLAMP_TWO:
                doClampTwo() ;
                break ;
            case WINDMILL_TWO:
                doWindmillTwo() ;
                break ;
            case UNCLAMP_TWO:
                doUnclampTwo() ;
                break ;
            case CLAMP_THREE:
                doClampThree() ;
                break ;
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "ClimbAction" ;
    }
 

    // my notes on states/actions
    // https://docs.google.com/spreadsheets/d/1VQvMyQ1WbQQrf9r2D5Rkg1IVWN1o5F0aYZadMqYttAs/edit#gid=0
    private boolean doIdle() {
        // currently does nothing
        // turn windmill 90 degrees until Clamp A is "up"?
       return true ;
    }

    private boolean doSquaring() {
        //waits for sensor to hit
        while (!sub_.isMidLeftTouched() || !sub_.isMidRightTouched()) {
            // TODO: refer to db and drive forward until this sensor is hit!
            // db_.setAction(driveforward) ;
        }
        return true;  
    }

    private boolean doClampOne() {
        if (sub_.isMidLeftTouched()) {
            while(!sub_.isMidRightTouched()) {
                // TODO: refer to db and drive opp wheel forward until this sensor is hit!
            }
            return true ;
        }
        else if (sub_.isMidRightTouched()) {
            while(!sub_.isMidLeftTouched()) {
                // TODO: refer to db and drive opp wheel forward until this sensor is hit!
            }
            return true ;
        }
        // TODO: turn off db
        sub_.setClampAClosed(true);
        state_start_time_ = sub_.getRobot().getTime() ;
        return true;
    }
     
    private boolean doWindmillOne() throws BadMotorRequestException, MotorRequestFailedException {
        // waits for 1st "clamping time"
        if (sub_.getRobot().getTime() - state_start_time_ > first_clamp_wait_) {
            sub_.setWindmill(first_windmill_power_);
        }
        return true;
    }

    private boolean doUnclampOne() throws BadMotorRequestException, MotorRequestFailedException {
        //waits for sensor to hit
        while (!sub_.isHighLeftTouched() || !sub_.isHighRightTouched()) {

        }
        state_start_time_ = sub_.getRobot().getTime() ;
        
        // wait for 1st "hooking/aligning time"
        if (sub_.getRobot().getTime() - state_start_time_ > first_align_wait_) {
            sub_.setWindmill(0.0) ;
            sub_.setClampAClosed(false);
        }
        state_start_time_ = sub_.getRobot().getTime() ;
        return true;
    }
    
    private boolean doClampTwo() {
        //wait for 1st "swinging time"
        if (sub_.getRobot().getTime() - state_start_time_ > first_swing_wait_) {
            sub_.setClampBClosed(true);
        }
        state_start_time_ = sub_.getRobot().getTime() ;
        return true;
    }
    
    private boolean doWindmillTwo() throws BadMotorRequestException, MotorRequestFailedException {
        if (sub_.getRobot().getTime() - state_start_time_ > second_clamp_wait_) {
            sub_.setWindmill(second_windmill_power_);
        }
        return true;
    }
    
    private boolean doUnclampTwo() throws BadMotorRequestException, MotorRequestFailedException {
        while (!sub_.isTraversalLeftTouched() || !sub_.isTraversalRightTouched()) {

        }
        state_start_time_ = sub_.getRobot().getTime() ;

        // wait for 2nd "hooking/aligning time"
        if (sub_.getRobot().getTime() - state_start_time_ > second_align_wait_) {
            sub_.setWindmill(0.0) ;
            sub_.setClampBClosed(false);
        }
        return true;
    }
     
    private boolean doClampThree() {
        //wait for 2nd "swinging time"
        if (sub_.getRobot().getTime() - state_start_time_ > second_swing_wait_) {
            sub_.setClampAClosed(true);
        }
        return true;
    }

}
