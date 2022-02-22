package frc.robot.conveyor;

import org.xero1425.base.actions.Action;

public class ConveyorExitAction extends Action {
    private ConveyorSubsystem sub_ ;
    private boolean onoff_ ;

    public ConveyorExitAction(ConveyorSubsystem sub, boolean onoff) {
        super(sub.getRobot().getMessageLogger()) ;

        sub_ = sub ;
        onoff_ = onoff ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;
        if (onoff_)
            sub_.openExit();
        else
            sub_.closeExit();

        setDone() ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;
    }

    public String toString(int indent) {
        return spaces(indent) + "ConveyorExitAction " + (onoff_? "true" : "false") ;
    }
    
}
