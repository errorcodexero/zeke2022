package frc.robot.climber;

import org.xero1425.base.actions.Action;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.tankdrive.TankDriveSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

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

    // todo: also take the gamepad/OI as a param so climber can disable it after it starts climbing
    public ClimbAction(ClimberSubsystem sub, TankDriveSubsystem db) throws BadParameterTypeException, MissingParameterException {

        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;
        db_ = db ;
        
        drive_action_power_ = sub.getSettingsValue("climbaction:drive_action_power").getDouble() ;

        first_windmill_power_ = sub.getSettingsValue("climbaction:first_windmill_power").getDouble() ;
        second_windmill_power_ = sub.getSettingsValue("climbaction:second_windmill_power").getDouble() ;

        first_clamp_wait_ = sub.getSettingsValue("climbaction:first_clamp_wait").getDouble() ;
        first_align_wait_ = sub.getSettingsValue("climbaction:first_align_wait").getDouble() ;
        first_swing_wait_ = sub.getSettingsValue("climbaction:first_swing_wait").getDouble() ;
        second_clamp_wait_ = sub.getSettingsValue("climbaction:second_clamp_wait").getDouble() ;
        second_align_wait_ = sub.getSettingsValue("climbaction:second_align_wait").getDouble() ;
        second_swing_wait_ = sub.getSettingsValue("climbaction:second_swing_wait").getDouble() ;

    }

    @Override
    public void start() throws Exception {
        super.start() ;
    }

    @Override
    public void run() throws BadMotorRequestException, MotorRequestFailedException {
        ClimbingStates prev = state_ ;

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
                // setDone() ;
                break ;
        }

        //
        // Butch: added this to print to the log file, we will need it plus I wanted to test
        //        the start of the climber model
        if (prev != state_) {
            MessageLogger logger = sub_.getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, sub_.getLoggerID()) ;
            logger.add("ClimberAction changed states: ") ;
            logger.addQuoted(prev.toString()).add(" --> ").addQuoted(state_.toString()) ;
            logger.endMessage();
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

    // notes on climber's states/actions
    // https://docs.google.com/spreadsheets/d/1VQvMyQ1WbQQrf9r2D5Rkg1IVWN1o5F0aYZadMqYttAs/edit#gid=0


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
            // - turn off db
            // - goto CLAMP_ONE state (we skip SQUARING)
        }
        else if (sub_.isMidLeftTouched() || sub_.isMidRightTouched()) {
            // The driver drove up to the bar and only one sensor hit in this
            // robot loop.
            // Do: 
            //   - disable driving from gamepad
            //   - turn off db
            //   - turn on motor on side of drivebase that has not hit the sensor
            //   - go to the SQUARING state
        }
        else {
            // Nothing happened to cause us to do anything, so we just stay in this
            // same state for the next robot loop.
        }
    }

    // doSquaring() - handles the SQUARING state
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

    
    // doClampOne() - handles the CLAMP_ONE state
    //    Exit Condition:
    //        Clamp A is set to be closed
    //
    //    Activities while in the state:
    //        set Clamp A to be closed
    //
    //    Activities when exit conditions are met:
    //        Go to the WINDMILL_ONE state
    //
    private void doClampOne() {
        sub_.setClampAClosed(true);
        state_start_time_ = sub_.getRobot().getTime() ;
    }
     
    // doWindmillOne() - handles the WINDMILL_ONE state
    //    Exit Condition:
    //        first windmill power is set to the windmills
    //
    //    Activities while in the state:
    //        wait for 1st clamping time to pass; first windmill power is set to the windmills
    //
    //    Activities when exit conditions are met:
    //        Go to the UNCLAMP_ONE state
    //
    private void doWindmillOne() throws BadMotorRequestException, MotorRequestFailedException {
        // waits for 1st "clamping time" to pass before setting windmill power
        if (sub_.getRobot().getTime() - state_start_time_ > first_clamp_wait_) {
            sub_.setWindmill(first_windmill_power_) ;
        }
    }

    // doUnclampOne() - handles the UNCLAMP_ONE state
    //    Exit Condition:
    //        Clamp A is set to open
    //
    //    Activities while in the state:
    //        wait for one of the sensors to hit
    //        then wait for aligning/hooking time of clamp B
    //        then stop windmill and unclamp A
    //
    //    Activities when exit conditions are met:
    //        Go to the CLAMP_TWO state
    //
    private void doUnclampOne() throws BadMotorRequestException, MotorRequestFailedException {
        //waits for sensor to hit
        if (sub_.isHighLeftTouched() || sub_.isHighRightTouched()) {
            state_start_time_ = sub_.getRobot().getTime() ;
        }

        // wait for 1st "hooking/aligning time"
        if (sub_.getRobot().getTime() - state_start_time_ > first_align_wait_) {
            sub_.setWindmill(0.0) ;
            sub_.setClampAClosed(false);
        }
        state_start_time_ = sub_.getRobot().getTime() ;
    }
    
    
    // doClampTwo() - handles the CLAMP_TWO state
    //    Exit Condition:
    //        Clamp B is set to closed
    //
    //    Activities while in the state:
    //        wait for the robot to "swing" a little on B's hook so it's comfortably hanging
    //        close clamp B
    //
    //    Activities when exit conditions are met:
    //        Go to the WINDMILL_TWO state
    //
    private void doClampTwo() {
        //wait for 1st "swinging time"
        if (sub_.getRobot().getTime() - state_start_time_ > first_swing_wait_) {
            sub_.setClampBClosed(true);
        }
        state_start_time_ = sub_.getRobot().getTime() ;
    }

    // doWindmillTwo() - handles the WINDMILL_TWO state
    //    Exit Condition:
    //        windmill is on; running at second_windmill_power
    //
    //    Activities while in the state:
    //        wait for the clamp B to fully close
    //        turn on the windmill for the 2nd time
    //
    //    Activities when exit conditions are met:
    //        Go to the UNCLAMP_TWO state
    //
    private void doWindmillTwo() throws BadMotorRequestException, MotorRequestFailedException {
        if (sub_.getRobot().getTime() - state_start_time_ > second_clamp_wait_) {
            sub_.setWindmill(second_windmill_power_);
        }
    }
    
    // doUnclampTwo() - handles the UNCLAMP_TWO state
    //    Exit Condition:
    //        unclamped/opened clamp b
    //
    //    Activities while in the state:
    //        wait for the sensors to touch traversal bar
    //        wait for hooking/aligning clamp A onto traversal bar
    //        open clamp B
    //
    //    Activities when exit conditions are met:
    //        Go to the CLAMP_THREE state
    //
    private void doUnclampTwo() throws BadMotorRequestException, MotorRequestFailedException {
        if (sub_.isTraversalLeftTouched() || sub_.isTraversalRightTouched()) {
            state_start_time_ = sub_.getRobot().getTime() ;
            // todo fix following code that relies on this state_start_time
        }
        
        // wait for 2nd "hooking/aligning time"
        if (sub_.getRobot().getTime() - state_start_time_ > second_align_wait_) {
            sub_.setWindmill(0.0) ;
            sub_.setClampBClosed(false);
        }
    }
     
    // doClampThree() - handles the CLAMP_THREE state
    //    Exit Condition:
    //        clamped/closed clamp A
    //
    //    Activities while in the state:
    //        wait for a swinging time so clamp A is comfortably hooked
    //        close Clamp A
    //
    //    Activities when exit conditions are met:
    //        Go to the DONE state
    //
    private void doClampThree() {
        //wait for 2nd "swinging time"
        if (sub_.getRobot().getTime() - state_start_time_ > second_swing_wait_) {
            sub_.setClampAClosed(true);
        }
    }

}
