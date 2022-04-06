package frc.robot.conveyor;

import org.xero1425.base.actions.Action;

public class ConveyorStopAction extends Action {
    private ConveyorSubsystem sub_ ;

    public ConveyorStopAction(ConveyorSubsystem sub) {
        super(sub.getRobot().getMessageLogger()) ;
        
        sub_ = sub ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.setStopRequest();
        setDone() ;
    }

    @Override
    public void run() {
    }

    @Override
    public void cancel() {
        super.cancel() ;
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "ConveyorStopAction" ;
    }
} ;
