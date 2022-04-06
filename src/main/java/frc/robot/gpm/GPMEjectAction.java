package frc.robot.gpm;

import org.xero1425.base.actions.Action;
import frc.robot.conveyor.ConveyorEjectAction;
import frc.robot.shooter.SetShooterAction;

public class GPMEjectAction extends Action {

    private GPMSubsystem sub_ ;
    private ConveyorEjectAction eject_ ;
    private SetShooterAction shooter_ ;

    public GPMEjectAction(GPMSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;        
        sub_ = sub ;

        eject_ = new ConveyorEjectAction(sub_.getConveyor()) ;
        shooter_ = new SetShooterAction(sub.getShooter(), -2000, -2000, 8.0) ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;
        sub_.getConveyor().setAction(eject_, true) ;
        sub_.getShooter().setAction(shooter_, true) ;
    }

    @Override
    public void run() {
        if (eject_.isDone()) {
            sub_.getShooter().stop() ;
            setDone() ;
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GPMEjectAction" ;
    }

    @Override
    public void cancel() {
        super.cancel(); ;
    }
}
