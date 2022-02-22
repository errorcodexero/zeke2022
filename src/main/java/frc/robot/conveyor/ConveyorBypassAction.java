package frc.robot.conveyor;

import org.xero1425.base.actions.Action;

public class ConveyorBypassAction extends Action {
    private ConveyorSubsystem sub_ ;
    private boolean bypass_ ;

    public ConveyorBypassAction(ConveyorSubsystem sub, boolean bypass) {
        super(sub.getRobot().getMessageLogger()) ;

        sub_ = sub ;
        bypass_ = bypass ;
    }

    @Override
    public void start() {
        sub_.setBypass(bypass_);
        setDone() ;
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "ConveyorBypassAction " + (bypass_ ? "true" : "false") ;
    }
}
