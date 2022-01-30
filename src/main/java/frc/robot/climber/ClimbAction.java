package frc.robot.climber;

import org.xero1425.base.actions.Action;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motorsubsystem.MotorPowerAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;

public class ClimbAction extends Action {

    private ClimberSubsystem sub_ ;
    private TankDriveSubsystem db_ ;

    private double drive_action_power_ ;

    // powers to windmills
    private double first_windmill_power_ ;
    private double second_windmill_power_ ;

    // timer to judge following delays off of
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
    public ClimbAction(ClimberSubsystem sub, TankDriveSubsystem db, double driveActionPower,
                        double firstWindmillPower, double secondWindmillPower, 
                        double firstClampWait, double firstAlignWait, double firstSwingWait,
                        double secondClampWait, double secondAlignWait, double secondSwingWait) {

        //
        // Butch: Let's not get all of these params from the constructor, but rather get them
        //        from the settings file.  For example.
        //
        // drive_action_power_ = sub.getSettingsValue("climbaction:drive_action_power") ;
        //

        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;
        db_ = db ;
        
        drive_action_power_ = driveActionPower ;

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

        // Butch: calling setDone() here will finish the action and it will do nothing.
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
 
    //
    // Butch: I would make the state method handlers void, they cannot really return anything useful. Also,
    // we can never use while loops like this as it will cause a single robot loop to exceed 20 ms.  We do
    // what checking we can do in a robot loop and then return control to the robot loop code.  I write some code
    // for the IDLE state (I know I picked an easy one) to demostrate.
    //
    // Note, this code is going to be complicated.  See how I commented each state method (e.g. doIdle()).  We need
    // this for each method in the state machine (e.g. doIdle(), doSquaring(), etc.)
    //

    // my notes on states/actions
    // https://docs.google.com/spreadsheets/d/1VQvMyQ1WbQQrf9r2D5Rkg1IVWN1o5F0aYZadMqYttAs/edit#gid=0

    //
    // doIdle() - handles the IDLE state
    //    Exit Condition: 
    //        Either the left or right mid sensors (or both) are triggered
    //
    //    Activities while in the state:
    //        None
    //
    //    Activities when exit conditions are met:
    //        Disable driving from the game pad
    //        Turn on the motor on the side of the drivebase that has not hit the sensor
    //        Go to the SQUARING state
    //
    private void doIdle() {

        if (sub_.isMidRightTouched() && sub_.isMidLeftTouched()) {
            // The driver drove up to the bar perfectly and both sensors
            // touched in the same robot loop.  Unlikely, but it could happen.
            // Do:
            // - disable game pad
            // - goto CLAMP_ONE state (we skip SQUARING)
        }
        else if (sub_.isMidLeftTouched() || sub_.isMidRightTouched()) {
            // The driver drove up to the bar and only one sensor hit in this
            // robot loop.
            // Do: 
            //   - disable driving from gamepad
            //   - turn on motor on side of drivebase that has not hit the sensor
            //   - go to the SQUARING state
        }
        else {
            // Nothing happened to cause us to do anything, so we just stay in this
            // same state for the next robot loop.
        }
    }

    // Butch:
    //    Exit Condition:
    //        Both mid sensors are triggered (we are squared to the bar)
    //
    //    Activities while in the state:
    //        None
    //
    //    Activities when exit conditions are met:
    //        Go to the CLAMP_ONE state
    //
    private void doSquaring() {

        while (!sub_.isMidLeftTouched() || !sub_.isMidRightTouched()) {
            // TODO: refer to db and drive forward until this sensor is hit!
            // db_.setPower(drive_action_power_, drive_action_power_) ;
        }
    }

    private void doClampOne() {
        if (sub_.isMidLeftTouched()) {
            while(!sub_.isMidRightTouched()) {
                // TODO: refer to db and drive opp wheel forward until this sensor is hit!
            }
        }
        else if (sub_.isMidRightTouched()) {
            while(!sub_.isMidLeftTouched()) {
                // TODO: refer to db and drive opp wheel forward until this sensor is hit!
            }
        }
        // TODO: turn off db
        sub_.setClampAClosed(true);
        state_start_time_ = sub_.getRobot().getTime() ;
    }
     
    private void doWindmillOne() throws BadMotorRequestException, MotorRequestFailedException {
        // waits for 1st "clamping time"
        if (sub_.getRobot().getTime() - state_start_time_ > first_clamp_wait_) {
            sub_.setWindmill(first_windmill_power_);
        }
    }

    private void doUnclampOne() throws BadMotorRequestException, MotorRequestFailedException {
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
    }
    
    private void doClampTwo() {
        //wait for 1st "swinging time"
        if (sub_.getRobot().getTime() - state_start_time_ > first_swing_wait_) {
            sub_.setClampBClosed(true);
        }
        state_start_time_ = sub_.getRobot().getTime() ;
    }
    
    private void doWindmillTwo() throws BadMotorRequestException, MotorRequestFailedException {
        if (sub_.getRobot().getTime() - state_start_time_ > second_clamp_wait_) {
            sub_.setWindmill(second_windmill_power_);
        }
    }
    
    private void doUnclampTwo() throws BadMotorRequestException, MotorRequestFailedException {
        while (!sub_.isTraversalLeftTouched() || !sub_.isTraversalRightTouched()) {

        }
        state_start_time_ = sub_.getRobot().getTime() ;

        // wait for 2nd "hooking/aligning time"
        if (sub_.getRobot().getTime() - state_start_time_ > second_align_wait_) {
            sub_.setWindmill(0.0) ;
            sub_.setClampBClosed(false);
        }
    }
     
    private void doClampThree() {
        //wait for 2nd "swinging time"
        if (sub_.getRobot().getTime() - state_start_time_ > second_swing_wait_) {
            sub_.setClampAClosed(true);
        }
    }

}
