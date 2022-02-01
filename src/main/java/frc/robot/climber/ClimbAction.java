package frc.robot.climber;

import org.xero1425.base.actions.Action;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.tankdrive.TankDrivePowerAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import frc.robot.climber.ClimberSubsystem.ChangeClampTo;

public class ClimbAction extends Action {

    private ClimberSubsystem sub_ ;
    private TankDriveSubsystem db_ ;
    private TankDrivePowerAction left_wheel_ ;
    private TankDrivePowerAction right_wheel_ ;
    private TankDrivePowerAction stop_db_ ;

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
        left_wheel_ = new TankDrivePowerAction(db_, drive_action_power_, 0.0) ;
        right_wheel_ = new TankDrivePowerAction(db_, 0.0, drive_action_power_) ;
        stop_db_ = new TankDrivePowerAction(db_, 0.0, 0.0) ;


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
            case CLAMP_TWO:
                doClampTwo() ;
                break ;
            case UNCLAMP_ONE:
                doUnclampOne() ;
                break ;
            case WINDMILL_TWO:
                doWindmillTwo() ;
                break ;
            case UNCLAMP_TWO:
                doClampThree() ;
                break ;
            case CLAMP_THREE:
                doUnclampTwo() ;
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

            // set clamp A => closed
            sub_.setClampA(ChangeClampTo.CLOSED) ;

            // - go to CLAMP_ONE state (we skip SQUARING)
            state_ = ClimbingStates.CLAMP_ONE ;
            // - get a time stamp to use in next method; this is to give time for clamp A to be closed
            state_start_time_ = sub_.getRobot().getTime() ;
        }
        else if (sub_.isMidLeftTouched() || sub_.isMidRightTouched()) {
            // The driver drove up to the bar and only one sensor hit in this
            // robot loop.
            // Do: 
            //   - disable driving from gamepad

            //   - turn on motor on side of drivebase that has not hit the sensor
            if (sub_.isMidLeftTouched()) {
                db_.setAction(right_wheel_) ;
            } else { // mid right touched
                db_.setAction(left_wheel_) ;
            }

            //   - go to the SQUARING state
            state_ = ClimbingStates.SQUARING ;
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
    //        Turn off drivebase 
    //        Go to the CLAMP_ONE stat        
    //
    private void doSquaring() {
        if (sub_.isMidLeftTouched() && sub_.isMidRightTouched()) { 
            // both sensors are touching

            // - turn off the db
            db_.setAction(stop_db_) ;

            // - clamp A state
            sub_.setClampA(ChangeClampTo.CLOSED);
            
            // - go to CLAMP_ONE state
            state_ = ClimbingStates.CLAMP_ONE ;
            // - get a time stamp to use in next method; this is to give time for clamp A to be closed
            state_start_time_ = sub_.getRobot().getTime() ;
        }
    }

    // doClampOne() - handles the CLAMP_ONE state
    //    Exit Condition:
    //        Clamp A is closed
    //        wait until one of the high sensors have been hit
    //
    //    Activities while in the state:
    //        Clamp A to be closed - wait for 1st clamping time to pass 
    //        start up the windmill
    //
    //    Activities when exit conditions are met:
    //        Go to the WINDMILL_ONE state
    //
    private void doClampOne() throws BadMotorRequestException, MotorRequestFailedException {
        // clamping clamp A; wait for the 2st clamping time to pass
        if (sub_.getRobot().getTime() - state_start_time_ > first_clamp_wait_) {
            sub_.setWindmill(first_windmill_power_) ;

            state_ = ClimbingStates.WINDMILL_ONE ;
        }
    }
     
    // doWindmillOne() - handles the WINDMILL_ONE state
    //    Exit Condition:
    //        Clamp A has unclamped
    //
    //    Activities while in the state:
    //        first windmill power is being set to the windmills
    //        then wait for aligning/hooking time of clamp B
    //        then stop windmill and unclamp A
    //
    //    Activities when exit conditions are met:
    //        Go to the UNCLAMP_ONE state
    //
    private void doWindmillOne() throws BadMotorRequestException, MotorRequestFailedException {
        // wait for 1st "hooking/aligning time" of clamp B; then unclamp A
        // - waits for high sensor to hit
        if (sub_.isHighLeftTouched() && sub_.isHighRightTouched()) {
            sub_.setWindmill(0.0) ;
            sub_.setClampB(ChangeClampTo.CLOSED);
            state_ = ClimbingStates.CLAMP_TWO ;
        }
        state_start_time_ = sub_.getRobot().getTime() ;
    }

    // doUnclampOne() - handles the UNCLAMP_ONE state
    //    Exit Condition:
    //        Clamp B is set to closed
    //
    //    Activities while in the state:
    //        wait for the robot to "swing" a little on B's clamp's hook so it's comfortably hanging
    //        close clamp B
    //
    //    Activities when exit conditions are met:
    //        Go to the CLAMP_TWO state
    //
    private void doClampTwo() {
        // wait for 1st "swinging time"
        if (sub_.getRobot().getTime() - state_start_time_ > first_swing_wait_) {
            sub_.setClampA(ChangeClampTo.OPEN);
            state_ = ClimbingStates.WINDMILL_TWO ;
        }
        state_start_time_ = sub_.getRobot().getTime() ;
    }
    
    // doClampTwo() - handles the CLAMP_TWO state
    //    Exit Condition:
    //        windmill is on; running at second_windmill_power
    //
    //    Activities while in the state:
    //        wait for the clamp B to fully close
    //        turn on the windmill for the 2nd time
    //
    //    Activities when exit conditions are met:
    //        Go to the WINDMILL_TWO state
    //
    private void doUnclampOne() throws BadMotorRequestException, MotorRequestFailedException {
        if (sub_.getRobot().getTime() - state_start_time_ > second_clamp_wait_) {
            sub_.setWindmill(second_windmill_power_);
        }
        if (sub_.isTraversalLeftTouched() || sub_.isTraversalRightTouched()) {
            state_start_time_ = sub_.getRobot().getTime() ;
            state_ = ClimbingStates.UNCLAMP_TWO ;
        }
    }

    // doWindmillTwo() - handles the WINDMILL_TWO state
    //    Exit Condition:
    //        unclamped/opened clamp b
    //
    //    Activities while in the state:
    //        wait for the sensors to touch traversal bar
    //        wait for hooking/aligning clamp A onto traversal bar
    //        open clamp B
    //
    //    Activities when exit conditions are met:
    //        Go to the UNCLAMP_TWO state
    //
    private void doWindmillTwo() throws BadMotorRequestException, MotorRequestFailedException {
        // wait for 1st "hooking/aligning time" of clamp B; then unclamp A
        // - waits for high sensor to hit
        if (sub_.isTraversalLeftTouched() && sub_.isTraversalRightTouched()) {
            sub_.setWindmill(0.0) ;
            sub_.setClampA(ChangeClampTo.CLOSED);
            state_ = ClimbingStates.CLAMP_THREE ;
        }
    }
    
    // doUnclampTwo() - handles the UNCLAMP_TWO state
    //    Exit Condition:
    //        clamped/closed clamp A
    //
    //    Activities while in the state:
    //        wait for a swinging time so clamp A is comfortably hooked
    //        close Clamp A
    //
    //    Activities when exit conditions are met:
    //        Go to the CLAMP_THREE state
    //
    private void doClampThree() {     
        state_start_time_ = sub_.getRobot().getTime() ;
        //wait for 2nd "swinging time"
        if (sub_.getRobot().getTime() - state_start_time_ > second_swing_wait_) {
            sub_.setClampB(ChangeClampTo.OPEN);
            state_ = ClimbingStates.CLAMP_THREE ;
        }
    }
     
    // doClampThree() - handles the CLAMP_THREE state
    //    Exit Condition:
    //        none
    //
    //    Activities while in the state:
    //        holding on for dear life 
    //
    //    Activities when exit conditions are met:
    //        never exits; not really
    //
    private void doUnclampTwo() throws BadMotorRequestException, MotorRequestFailedException {
        // basically do nothing and wait for match to end
        
        // continuously set clamp A to "closed"?
    }

}
