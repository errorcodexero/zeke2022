package frc.robot.climber;

import org.xero1425.base.actions.Action;

public class ClimbAction extends Action {

    ClimberSubsystem sub_ ;

    public ClimbAction(ClimberSubsystem sub) {
        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        // turn windmill 90 degrees until Clamp A is "up"

        setDone() ;
    }

    @Override
    public void run() {

        // PUT THIS PART DOWN IN ANOTHER SUBSYSTEM     
        // if mid_left or med_right touch the bar
            // drive non-touched side foward until both are hitting m bar

        // THIS PART STAYS HERE
        // Clamp A's
        // turn windmills ~ 100 degrees
        // if sensors have been touched... etc etc

    }

    @Override
    public void cancel() {
        super.cancel() ;
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "ClimbAction" ;
    }
    
}
