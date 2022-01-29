package frc.robot.climber;

import org.xero1425.base.actions.Action;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.tankdrive.TankDriveAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;

public class ClimbAction extends Action {

    private ClimberSubsystem sub_ ;
    private TankDriveSubsystem db_ ;

    private double state_start_time_ ;
    
    private double first_clamp_wait_ ;
    private double second_clamp_wait_ ;
    private double first_windmill_power_ ;
    private double second_windmill_power_ ;
    
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
                        double firstClampWait, double secondClampWait, 
                        double firstWindmillPower, double secondWindmillPower) {
        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;
        db_ = db ;

        first_clamp_wait_ = firstClampWait ;
        second_clamp_wait_ = secondClampWait ;

        first_windmill_power_ = firstWindmillPower ;
        second_windmill_power_ = secondWindmillPower ;
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
        // turn windmill 90 degrees until Clamp A is "up"
       return true ;
    }

    private boolean doSquaring() {
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
        if (sub_.getRobot().getTime() - state_start_time_ > first_clamp_wait_) {
            sub_.setWindmill(first_windmill_power_);
        }
        return true;
    }

    private boolean doUnclampOne() {
        // add stuff :)
        return true;
    }
    
    private boolean doClampTwo() {
        // add stuff :)
        return true;
    }
    
    private boolean doWindmillTwo() {
        // add stuff :)
        return true;
    }
    
    private boolean doUnclampTwo() {
        // add stuff :)
        return true;
    }
     
    private boolean doClampThree() {
        // add stuff :)
        return true;
    }

}

    // if mid_left or med_right touch the bar
    // drive non-touched side foward UNTIL both are hitting m bar
        // Clamp A
        // turn windmills ~ 100 degrees
        // turn windmills slowly UNTIL sensors high left and high right have been touched 
            // release clamp As
            // clamp B
            // turn windmills ~100 degrees
            // turn windmills slowly UNTIL traversal left and traversal right have been touched
                // release clamp Bs
                // wait until robot's down a decent amount to recuse strain/stress
                // clamp As