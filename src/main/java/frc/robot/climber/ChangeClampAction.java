package frc.robot.climber;

import org.xero1425.base.actions.Action;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.climber.ClimberSubsystem.GrabberState;
import frc.robot.climber.ClimberSubsystem.WhichClamp;

public class ChangeClampAction extends Action {

    private ClimberSubsystem sub_ ;
    private WhichClamp which_clamp_ ;
    private GrabberState clamp_setting_ ;

    private double state_start_time_ ;
    private double wait_time_ ;

    // see what the input is - take in 2 variables
        // which clamp (create an enum for clamp A or clamp B)
        // what to set it to (use existing subsystem enum for open/closed)
    //set that clamp to that position using methods created in the climber subsystem
    public ChangeClampAction(ClimberSubsystem sub, WhichClamp which_clamp, GrabberState clamp_setting) 
                            throws BadParameterTypeException, MissingParameterException {
        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;
        which_clamp_ = which_clamp ;
        clamp_setting_ = clamp_setting ;

        wait_time_ = sub_.getSettingsValue("change-clamp-action:wait_time").getDouble() ;

    }

    @Override
    public void start() throws Exception {
        super.start() ;
        sub_.changeClamp(which_clamp_, clamp_setting_) ;
        state_start_time_ = sub_.getRobot().getTime() ;
    }

    @Override
    public void run() {
        // wait for an elapsed time
        if (sub_.getRobot().getTime() - state_start_time_  >= wait_time_) {
            setDone() ;
        }

    }

    @Override
    public void cancel() {
        super.cancel() ;
        
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "ChangeClampAction " + which_clamp_.toString() + " " + clamp_setting_.toString() ;
    }
 
}
