package frc.robot.conveyor;

import org.xero1425.base.actions.Action;

public class ConveyorShootAction extends Action {
    public ConveyorShootAction(ConveyorSubsystem sub) {
        super(sub.getRobot().getMessageLogger()) ;
        
        sub_ = sub ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.setShootMode() ;
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
        String ret = prefix(indent) + "ConveyorShootAction" ;
        ret += " shooter = " + Double.toString(shooter_) ;

        return ret ;
    }

    private ConveyorSubsystem sub_ ;
    private double shooter_ ;
} ;