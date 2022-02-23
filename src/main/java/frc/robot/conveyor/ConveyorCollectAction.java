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

        if (sub_.getBallCount() == 2) {
            setDone();
        }
        else {
            sub_.setCollectMode() ;
        }
    }

    @Override
    public void run() {
        if (sub_.getBallCount() == 2) {
            sub_.setStopCollect();
            setDone() ;
        }
    }

    @Override
    public void cancel() {
        if (!isDone()) {
            sub_.setStopCollect();
        }
        super.cancel() ;
    }

    @Override
    public String toString(int indent) {
        String ret = prefix(indent) + "ConveyorCollectAction" ;
        return ret ;
    }

    private ConveyorSubsystem sub_ ;
} ;