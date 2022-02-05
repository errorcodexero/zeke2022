package frc.robot.conveyor;

import org.xero1425.base.actions.Action;

public class ConveyorCollectAction extends Action {
    public ConveyorCollectAction(ConveyorSubsystem sub, double intake, double shooter) {
        super(sub.getRobot().getMessageLogger()) ;
        
        sub_ = sub ;
        intake_ = intake ;
        shooter_ = shooter ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.setMotorsPower(intake_, shooter_) ;
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
        String ret = prefix(indent) + "ConveyorCollectAction" ;
        ret += " intake = " + Double.toString(intake_) ;
        ret += " shooter = " + Double.toString(shooter_) ;

        return ret ;
    }

    private ConveyorSubsystem sub_ ;
    private double intake_ ;
    private double shooter_ ;
} ;