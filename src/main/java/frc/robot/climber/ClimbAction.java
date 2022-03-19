package frc.robot.climber;

import org.xero1425.base.actions.Action;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.tankdrive.TankDrivePowerAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.PIDCtrl;

import frc.robot.climber.ClimberSubsystem.GrabberState;
import frc.robot.climber.ClimberSubsystem.WhichClamp;
import frc.robot.zekeoi.ZekeOISubsystem;

public class ClimbAction extends Action {

    private ClimberSubsystem sub_ ;
    private TankDriveSubsystem db_ ;
    private ZekeOISubsystem oi_ ;
    private TankDrivePowerAction stop_db_ ;

    private double backup_target_mid_high_ ;
    private double backup_time_mid_high_ ;
    private double backup_target_high_traverse_ ;
    private double backup_time_high_traverse_ ;
    private PIDCtrl backup_mid_to_high_pid_ ;
    private PIDCtrl backup_high_to_traverse_pid_ ;
    private double backup_threshold_ ;

    private double squaring_touch_duration_ ;

    private double target_high_ ;
    private double target_traverse_ ;
    private ClimbPowerController windmill_mid_to_high_ctrl_ ;
    private ClimbPowerController windmill_high_to_traverse_ctrl_ ;

    private double drive_action_power_ ;
    private boolean stop_when_safe_ ;
    private boolean set_auto_drive_ ;

    // timer to judge following delays off of
    private double state_start_time_ ;

    private double hold_power_ ;

    private double clamp_wait_time_ ;
    private double unclamp_unloaded_wait_time_ ;
    private double unclamp_loaded_wait_time_ ;

    private boolean past_no_return_ ;

    private enum ClimbingStates {
        IDLE,
        OPEN_A_FOR_MID,
        WAIT_LEFT_OR_RIGHT_MID,
        SQUARING,
        CLOSE_A_ON_MID,
        OPEN_B_FOR_HIGH,
        WINDMILL_TO_HIGH_BAR,
        CLOSE_B_ON_HIGH,
        BACKUP_MID_TO_HIGH,
        OPEN_A_ON_MID,
        WINDMILL_TO_TRAVERSE_BAR,
        CLOSE_A_ON_TRAVERSE,
        BACKUP_HIGH_TO_TRAVERSE,
        COMPLETE
    }

    private ClimbingStates state_ = ClimbingStates.IDLE ;

    // todo: also take the gamepad/OI as a param so climber can disable it after it starts climbing
    public ClimbAction(ClimberSubsystem sub, TankDriveSubsystem db, ZekeOISubsystem oi) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;
        db_ = db ;
        oi_ = oi ;

        drive_action_power_ = sub.getSettingsValue("climbaction:drive-action-power").getDouble() ;
        stop_db_ = new TankDrivePowerAction(db_, 0.0, 0.0) ;

        clamp_wait_time_ = sub.getSettingsValue("climbaction:clamp-wait-time").getDouble() ;
        unclamp_unloaded_wait_time_ = sub.getSettingsValue("climbaction:unclamp-unloaded-wait-time").getDouble() ;
        unclamp_loaded_wait_time_ = sub.getSettingsValue("climbaction:unclamp-loaded-wait-time").getDouble() ;

        squaring_touch_duration_ = sub.getSettingsValue("climbaction:squaring-touch-duration").getDouble() ;

        hold_power_ = sub.getSettingsValue("climbaction:hold_power").getDouble() ;

        backup_target_mid_high_ = sub.getSettingsValue("climbaction:backup-target-mid-high").getDouble() ;
        backup_target_high_traverse_ = sub.getSettingsValue("climbaction:backup-target-high-traverse").getDouble() ;
        backup_threshold_ = sub.getSettingsValue("climbaction:backup-threshold").getDouble() ;

        backup_mid_to_high_pid_ = new PIDCtrl(sub.getRobot().getSettingsSupplier(), "subsystems:climber:climbaction:backup-mid-high", false) ;
        backup_high_to_traverse_pid_ = new PIDCtrl(sub.getRobot().getSettingsSupplier(), "subsystems:climber:climbaction:backup-high-traverse", false) ;

        target_high_ = sub.getSettingsValue("climbaction:target-high").getDouble() ;
        target_traverse_ = sub.getSettingsValue("climbaction:target-traverse").getDouble() ;
        double startrange = sub.getSettingsValue("climbaction:start-range").getDouble() ;
        double maxpower = sub.getSettingsValue("climbaction:max-windmill-power").getDouble() ;
        double finishpower = sub.getSettingsValue("climbaction:finish-power").getDouble() ;
        double finishrange = sub.getSettingsValue("climbaction:finish-range").getDouble() ;

        windmill_mid_to_high_ctrl_ = new ClimbPowerController(target_high_, startrange, maxpower, finishpower, finishrange) ;
        windmill_high_to_traverse_ctrl_ = new ClimbPowerController(target_traverse_, startrange, maxpower, finishpower, finishrange) ;

        stop_when_safe_ = false ;
        past_no_return_ = false ;
        set_auto_drive_ = false ;
        state_ = ClimbingStates.IDLE ;
    }

    public void setAutomaticDrive() {
        set_auto_drive_ = true ;
    }

    public void stopWhenSafe() {
        stop_when_safe_ = true ;
    }

    public boolean pastPointNoReturn() {
        return past_no_return_ ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        if (past_no_return_) {
            setDone() ;
        }
        else {
            stop_when_safe_ = false ;
            state_ = ClimbingStates.OPEN_A_FOR_MID ;
        }
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (stop_when_safe_ && !past_no_return_) {
            //
            // If we have not gone past the point of no return, and a stop is
            // requested, we then just complete the action
            //
            setDone() ;
            return ;
        }
        
        ClimbingStates prev = state_ ;

        switch (state_) {
            case IDLE:
                state_ = ClimbingStates.OPEN_A_FOR_MID ;
                break ;
            case OPEN_A_FOR_MID:
                doOpenAforMid() ;
                break ;
            case WAIT_LEFT_OR_RIGHT_MID:
                doWaitLeftOrRightMid() ;
                break ;
            case SQUARING:
                doSquaring() ;
                break ;
            case CLOSE_A_ON_MID:
                doCloseAOnMid() ;
                break ;
            case OPEN_B_FOR_HIGH:
                doOpenBForHigh() ;
                break ;
            case WINDMILL_TO_HIGH_BAR:
                doWindmillToHighBar() ;
                break ;
            case BACKUP_MID_TO_HIGH:
                doBackupMidToHigh() ;
                break ;
            case CLOSE_B_ON_HIGH:
                doCloseBOnHigh() ;
                break ;
            case OPEN_A_ON_MID:
                doOpenAOnMid() ;
                break ;
            case WINDMILL_TO_TRAVERSE_BAR:
                doWindmillToTraverse() ;
                break ;
            case CLOSE_A_ON_TRAVERSE:
                doCloseAOnTraverse();
                break ;
            case BACKUP_HIGH_TO_TRAVERSE:
                doBackupHighToTraverse() ;
                break ;
            case COMPLETE:
                doComplete() ;
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
            logger.add("encoder", sub_.getWindmillMotor().getPosition()) ;
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

    private void doOpenAforMid() {
        sub_.changeClamp(WhichClamp.CLAMP_A, GrabberState.OPEN);
        state_ = ClimbingStates.WAIT_LEFT_OR_RIGHT_MID ;
    }

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
    private void doWaitLeftOrRightMid() {
        if (sub_.isLeftATouched() || sub_.isRightATouched() || set_auto_drive_) {
            //
            // Disable driving from gamepad            
            //
            oi_.getGamePad().disable();

            // Go to the squaring state which will manage the drive base power
            state_ = ClimbingStates.SQUARING ;
        }
    }

    // doSquaring() - handles the SQUARING state
    //    Exit Condition:
    //        Both mid sensors are triggered (we are squared to the bar)
    //
    //    Activities while in the state:
    //        Move the side of the drivebase that is not at the bar to the bar
    //
    //    Activities when exit conditions are met:
    //        Turn off drivebase 
    //
    private void doSquaring() {
        // both sensors are touching
        if (sub_.isLeftATouched() && sub_.leftADuration() > squaring_touch_duration_ && sub_.isRightATouched() && sub_.rightADuration() > squaring_touch_duration_) {
            // - turn off the db
            db_.setAction(stop_db_) ;

            // - clamp A state
            sub_.changeClamp(WhichClamp.CLAMP_A, GrabberState.CLOSED);
            
            // - get a time stamp to use in next method; this is to give time for clamp A to be closed
            state_start_time_ = sub_.getRobot().getTime() ;

            // - go to CLAMP_ONE state
            state_ = ClimbingStates.CLOSE_A_ON_MID ;

            // Beyond this point, this action cannot be stopped
            past_no_return_ = true ;
        }
        else {
            double left = 0.0 ;
            double right = 0.0 ;

            if (sub_.isLeftATouched() == false || sub_.leftADuration() <= squaring_touch_duration_)
                left =  drive_action_power_ ;

            if (sub_.isRightATouched() == false || sub_.rightADuration() <= squaring_touch_duration_)
                right = drive_action_power_ ;

            TankDrivePowerAction pa = new TankDrivePowerAction(db_, left, right) ;
            db_.setAction(pa) ;
        }
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
    private void doCloseAOnMid() throws BadMotorRequestException, MotorRequestFailedException {
        // clamping clamp A; wait for the 1st clamping time to pass
        if (sub_.getRobot().getTime() - state_start_time_ > clamp_wait_time_) {

            state_start_time_ = sub_.getRobot().getTime() ;       
            sub_.changeClamp(WhichClamp.CLAMP_B, GrabberState.OPEN);     
            state_ = ClimbingStates.OPEN_B_FOR_HIGH ;
        }
    }

    private void doOpenBForHigh() throws BadMotorRequestException, MotorRequestFailedException {
        if (sub_.getRobot().getTime() - state_start_time_ > unclamp_unloaded_wait_time_) {
            
            // Set the default action for the windmill to null so the hold action stops working
            sub_.getWindmillMotor().setDefaultAction(null);

            // Start the windmill toward the high bar
            windmill_mid_to_high_ctrl_.start() ;
            
            state_ = ClimbingStates.WINDMILL_TO_HIGH_BAR ;
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
    private void doWindmillToHighBar() throws BadMotorRequestException, MotorRequestFailedException {
        // - waits for high sensor to hit
        if (sub_.isLeftBTouched() && sub_.isRightBTouched()) {
            sub_.getWindmillMotor().setPower(hold_power_) ;
            
            sub_.changeClamp(WhichClamp.CLAMP_B, GrabberState.CLOSED);
            
            state_start_time_ = sub_.getRobot().getTime() ;
            state_ = ClimbingStates.CLOSE_B_ON_HIGH ;
        }
        else {
            double out = windmill_mid_to_high_ctrl_.getOutput(sub_.getWindmillMotor().getPosition()) ;
            sub_.getWindmillMotor().setPower(out);
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
    private void doCloseBOnHigh() throws Exception {
        // wait for 2nd "clamp time"
        if (sub_.getRobot().getTime() - state_start_time_ > clamp_wait_time_) {
            double out = backup_mid_to_high_pid_.getOutput(backup_target_mid_high_, sub_.getWindmillMotor().getPosition(), sub_.getRobot().getDeltaTime()) ;
            sub_.getWindmillMotor().setPower(out);

            state_start_time_ = sub_.getRobot().getTime() ;
            state_ = ClimbingStates.BACKUP_MID_TO_HIGH ;
        }
    }

    private void doBackupMidToHigh() {

        double err = Math.abs(sub_.getWindmillMotor().getPosition() - backup_target_mid_high_) ;
        if (err < backup_threshold_ || sub_.getRobot().getTime() - state_start_time_ > backup_time_mid_high_) {
            if (err >= backup_threshold_) {
                MessageLogger logger = sub_.getRobot().getMessageLogger() ;
                logger.startMessage(MessageType.Warning).add("backup step ended due to timeout, not position").endMessage();
            }

            if (!sub_.isLeftBTouched() || !sub_.isRightBTouched()) {
                state_ = ClimbingStates.COMPLETE ;
            }
            else {
                sub_.changeClamp(WhichClamp.CLAMP_A, GrabberState.OPEN);   
                state_start_time_ = sub_.getRobot().getTime() ;

                if (stop_when_safe_)
                    state_ = ClimbingStates.COMPLETE ;
                else {
                    state_ = ClimbingStates.OPEN_A_ON_MID ;
                }
            }            
        }
        else {
            double out = backup_mid_to_high_pid_.getOutput(backup_target_mid_high_, sub_.getWindmillMotor().getPosition(), sub_.getRobot().getDeltaTime()) ;
            sub_.getWindmillMotor().setPower(out);
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
    private void doOpenAOnMid() throws BadMotorRequestException, MotorRequestFailedException {
        // wait for clamp A to completely unclamp
        if (sub_.getRobot().getTime() - state_start_time_ > unclamp_loaded_wait_time_) {
            windmill_high_to_traverse_ctrl_.start() ;
            state_ = ClimbingStates.WINDMILL_TO_TRAVERSE_BAR ;
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
    private void doWindmillToTraverse() throws BadMotorRequestException, MotorRequestFailedException {
        // - waits for high sensor to hit
        if (sub_.isLeftATouched() && sub_.isRightATouched()) {

            //
            // Set windmill to a holding power, to hold use against the bar while the grabberse close
            //
            sub_.getWindmillMotor().setPower(hold_power_) ;

            // sets clamp A to be closed
            sub_.changeClamp(WhichClamp.CLAMP_A, GrabberState.CLOSED);

            state_start_time_ = sub_.getRobot().getTime() ;
            state_ = ClimbingStates.CLOSE_A_ON_TRAVERSE ;
        }
        else {
            double out = windmill_high_to_traverse_ctrl_.getOutput(sub_.getWindmillMotor().getPosition()) ;
            sub_.getWindmillMotor().setPower(out);            
        }
    }
    
    //
    // doClampThree() - handles the UNCLAMP_TWO state
    //
    //    Exit Condition:
    //        clamp A has had sufficient time to close
    //
    //    Activities while in the state:
    //        clamp A fully closes around the traversal bar
    //
    //    Activities when exit conditions are met:
    //        Go to the UNCLAMP_TWO state
    //
    private void doCloseAOnTraverse() throws Exception {     
        //wait for 2nd "unclamping time"
        if (sub_.getRobot().getTime() - state_start_time_ > unclamp_loaded_wait_time_) {
            double out = backup_high_to_traverse_pid_.getOutput(backup_target_high_traverse_, sub_.getWindmillMotor().getPosition(), sub_.getRobot().getDeltaTime()) ;
            sub_.getWindmillMotor().setPower(out);

            state_start_time_ = sub_.getRobot().getTime() ;
            state_ = ClimbingStates.BACKUP_HIGH_TO_TRAVERSE ;
        }
    }

    private void doBackupHighToTraverse() {

        double err = Math.abs(sub_.getWindmillMotor().getPosition() - backup_target_high_traverse_) ;
        if (err < backup_threshold_ || sub_.getRobot().getTime() - state_start_time_ > backup_time_high_traverse_) {
            if (!sub_.isLeftATouched() || !sub_.isRightATouched()) {
                state_ = ClimbingStates.COMPLETE ;
            }
            else {
                sub_.changeClamp(WhichClamp.CLAMP_B, GrabberState.OPEN);
                state_ = ClimbingStates.COMPLETE ;
            }
        }
        else {
            double out = backup_high_to_traverse_pid_.getOutput(backup_target_high_traverse_, sub_.getWindmillMotor().getPosition(), sub_.getRobot().getDeltaTime()) ;
            sub_.getWindmillMotor().setPower(out);
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
    private void doComplete() throws BadMotorRequestException, MotorRequestFailedException {
        setDone() ;
    }
}
