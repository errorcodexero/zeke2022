package frc.robot.intake;

import org.xero1425.base.actions.Action;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class ZekeIntakeOffAction extends Action {
    private ZekeIntakeSubsystem subsystem_;

    public ZekeIntakeOffAction(ZekeIntakeSubsystem subsystem) throws BadParameterTypeException, MissingParameterException {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        subsystem_.setLeftCollectorPower(0);
        subsystem_.setRightCollectorPower(0);
        subsystem_.retractIntake();
        setDone();
    }

    @Override
    public void run() throws Exception {
        super.run() ;
    }

    @Override
    public void cancel() {
        super.cancel() ;
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "IntakeOffAction" ;
    }
    
}
