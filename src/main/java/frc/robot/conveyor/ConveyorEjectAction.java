package frc.robot.conveyor;

import org.xero1425.base.actions.Action;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class ConveyorEjectAction extends Action {
    private ConveyorSubsystem sub_ ;
    private double start_ ;
    private double duration_ ;
    private double power_ ;

    public ConveyorEjectAction(ConveyorSubsystem sub) throws BadParameterTypeException, MissingParameterException {
        super(sub.getRobot().getMessageLogger()) ;

        sub_ = sub ;
        duration_ = sub_.getSettingsValue("eject-action:duration").getDouble() ;
        power_ = sub_.getSettingsValue("eject-action:power").getDouble() ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.setBypass(true);
        start_ = sub_.getRobot().getTime() ;
        sub_.setMotorsPower(power_, power_) ;
        sub_.resetBallCount() ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        if (sub_.getRobot().getTime() - start_ > duration_) {
            sub_.setMotorsPower(0.0, 0.0);
            sub_.setBypass(false) ;
        }
    }

    @Override
    public void cancel() {
        super.cancel();
        try {
            sub_.setMotorsPower(0.0, 0.0);
        }
        catch(Exception ex) {
        }
        sub_.setBypass(false) ;
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "ConveyorEjectAction" ;
    }
}
