package frc.robot.gpm.intake;

import org.xero1425.base.actions.Action;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class ZekeIntakeOnAction extends Action {
    private ZekeIntakeSubsystem subsystem_;
    private double collector_motor_a_power_;
    private double collector_motor_b_power_;

    public ZekeIntakeOnAction(ZekeIntakeSubsystem subsystem) throws BadParameterTypeException, MissingParameterException {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;
        collector_motor_a_power_ = subsystem_.getSettingsValue("hw:collector:motor-a:power").getDouble();
        collector_motor_b_power_ = subsystem_.getSettingsValue("hw:collector:motor-b:power").getDouble();
        
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        subsystem_.setSolenoidPower(true);
        subsystem_.setCollectorPower(collector_motor_a_power_, collector_motor_b_power_);
    }
    
    @Override
    public void run() throws Exception {
        super.run() ;
    }

    @Override
    public void cancel() {
        super.cancel();

        try {
            subsystem_.setSolenoidPower(false);
            subsystem_.setCollectorPower(0.0,0.0);
        } 
        catch(Exception ex) {
        }
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "IntakeOnAction" ;
    }

}