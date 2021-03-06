package frc.robot.conveyor;

import org.xero1425.base.actions.Action;

public class ConveyorCollectAction extends Action {
    public ConveyorCollectAction(ConveyorSubsystem sub) {
        super(sub.getRobot().getMessageLogger()) ;
        
        sub_ = sub ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        if (sub_.isFull()) {
            setDone();
        }
        else {
            sub_.setCollectMode() ;
        }
    }

    @Override
    public void run() {
        if (sub_.isFull()) {
            sub_.setStopRequest();
            setDone() ;
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;

        if (!isDone()) {
            sub_.setStopRequest();
        }
    }

    @Override
    public String toString(int indent) {
        String ret = prefix(indent) + "ConveyorCollectAction" ;
        return ret ;
    }

    private ConveyorSubsystem sub_ ;
} ;