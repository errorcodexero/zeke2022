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
import frc.robot.climber.ClimberSubsystem.SetWindmillTo;

public class ClimbAction extends Action {

    private ClimberSubsystem sub_ ;
    private TankDriveSubsystem db_ ;
    private TankDrivePowerAction left_wheel_ ;
    private TankDrivePowerAction right_wheel_ ;
    private TankDrivePowerAction stop_db_ ;

    private double drive_action_power_ ;

    // timer to judge following delays off of
    private double state_start_time_ ;

    // delay times between clamping/unclamping sections
    // first/second = whether it's between mid-high or high-traversal
    // clamp = for the clamping to finish before windmill starts going around
    // unclamp = for the previous clamp to let go of the bar
    private double first_clamp_wait_ ;
    private double first_unclamp_wait_ ;
    private double second_clamp_wait_ ;
    private double second_unclamp_wait_ ;

    
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

        first_clamp_wait_ = sub.getSettingsValue("climbaction:first_clamp_wait").getDouble() ;
        first_unclamp_wait_ = sub.getSettingsValue("climbaction:first_unclamp_wait").getDouble() ;
        second_clamp_wait_ = sub.getSettingsValue("climbaction:second_clamp_wait").getDouble() ;
        second_unclamp_wait_ = sub.getSettingsValue("climbaction:second_unclamp_wait").getDouble() ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        state_ = ClimbingStates.IDLE ;
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
                doUnclampTwo() ;
                break ;
            case CLAMP_THREE:
                doClampThree();
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

            // - get a time stamp to use in next method; this is to give time for clamp A to be closed
            state_start_time_ = sub_.getRobot().getTime() ;
            // - go to CLAMP_ONE state (we skip SQUARING)
            state_ = ClimbingStates.CLAMP_ONE ;
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
        // both sensors are touching
        if (sub_.isMidLeftTouched() && sub_.isMidRightTouched()) { 
            // - turn off the db
            db_.setAction(stop_db_) ;

            // - clamp A state
            sub_.setClampA(ChangeClampTo.CLOSED);
            
            // - get a time stamp to use in next method; this is to give time for clamp A to be closed
            state_start_time_ = sub_.getRobot().getTime() ;
            // - go to CLAMP_ONE state
            state_ = ClimbingStates.CLAMP_ONE ;
        }
        // if one sensor is touching ... 
    }

    // doClampOne() - handles the CLAMP_ONE state
    //    Exit Condition:
    //       clamp A has had enough time to fully clamp
    //
    //    Activities while in the state:
    //       clamping/closing clamp A onto the mid bar
    //
    //    Activities when exit conditions are met:
    //        Go to the WINDMILL_ONE state
    //
    private void doClampOne() throws BadMotorRequestException, MotorRequestFailedException {
        // clamping clamp A; wait for the 1st clamping time to pass
        if (sub_.getRobot().getTime() - state_start_time_ > first_clamp_wait_) {
            sub_.setWindmill(SetWindmillTo.FORWARDS) ;

            state_ = ClimbingStates.WINDMILL_ONE ;
        }
    }
     
    // doWindmillOne() - handles the WINDMILL_ONE state
    //    Exit Condition:
    //       both sensors have contacted the high bar
    //
    //    Activities while in the state:
    //       windmilling up to high bar from mid
    //
    //    Activities when exit conditions are met:
    //        Go to the CLAMP_TWO state
    //
    private void doWindmillOne() throws BadMotorRequestException, MotorRequestFailedException {
        // - waits for high sensor to hit
        if (sub_.isHighLeftTouched() && sub_.isHighRightTouched()) {
            sub_.setWindmill(SetWindmillTo.OFF) ;
            sub_.setClampB(ChangeClampTo.CLOSED);
            
            state_start_time_ = sub_.getRobot().getTime() ;
            state_ = ClimbingStates.CLAMP_TWO ;
        }
    }

    // doUnclampOne() - handles the UNCLAMP_ONE state
    //    Exit Condition:
    //       clamp B has had enough time to clamp to high bar
    //
    //    Activities while in the state:
    //        clamping clamp B to high bar
    //
    //    Activities when exit conditions are met:
    //        Go to the UNCLAMP_ONE state
    //
    private void doClampTwo() {
        // wait for 2nd "clamp time"
        if (sub_.getRobot().getTime() - state_start_time_ > second_clamp_wait_) {
            sub_.setClampA(ChangeClampTo.OPEN);
            
            state_start_time_ = sub_.getRobot().getTime() ;
            state_ = ClimbingStates.UNCLAMP_ONE ;
        }
    }
    
    // doClampTwo() - handles the CLAMP_TWO state
    //    Exit Condition:
    //        clamp A has had enough time to clamp to high bar
    //
    //    Activities while in the state:
    //        unclamping clamp A from mid bar
    //        setting the windmill to run "backwards"; aka opp direction of the first windmilling
    //
    //    Activities when exit conditions are met:
    //        Go to the WINDMILL_TWO state
    //
    private void doUnclampOne() throws BadMotorRequestException, MotorRequestFailedException {
        // wait for clamp A to completely unclamp
        if (sub_.getRobot().getTime() - state_start_time_ > first_unclamp_wait_) {
            sub_.setWindmill(SetWindmillTo.FORWARDS);
        }
    }

    // doWindmillTwo() - handles the WINDMILL_TWO state
    //    Exit Condition:
    //        if both sensors touch the traversal bar
    //
    //    Activities while in the state:
    //        windmilling from the high bar to the traversal bar 
    //
    //    Activities when exit conditions are met:
    //        Go to the CLAMP_THREE state
    //
    private void doWindmillTwo() throws BadMotorRequestException, MotorRequestFailedException {
        // - waits for high sensor to hit
        if (sub_.isTraversalLeftTouched() && sub_.isTraversalRightTouched()) {
            // turns off windmill
            sub_.setWindmill(SetWindmillTo.OFF) ;
            // sets clamp A to be closed
            sub_.setClampA(ChangeClampTo.CLOSED);

            state_start_time_ = sub_.getRobot().getTime() ;
            state_ = ClimbingStates.CLAMP_THREE ;
        }
    }
    
    // doUnclampTwo() - handles the UNCLAMP_TWO state
    //    Exit Condition:
    //        clamp A has had sufficient time to close
    //
    //    Activities while in the state:
    //        clamp A fully closes around the traversal bar
    //
    //    Activities when exit conditions are met:
    //        Go to the UNCLAMP_TWO state
    //
    private void doClampThree() {     
        //wait for 2nd "unclamping time"
        if (sub_.getRobot().getTime() - state_start_time_ > second_unclamp_wait_) {
            sub_.setClampB(ChangeClampTo.OPEN);

            state_ = ClimbingStates.UNCLAMP_TWO ;
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
