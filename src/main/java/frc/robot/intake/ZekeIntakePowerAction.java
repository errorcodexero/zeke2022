package frc.robot.intake;

import org.xero1425.base.actions.Action;

public class ZekeIntakePowerAction extends Action {
    private ZekeIntakeSubsystem subsystem_;

    public enum IntakeMotor {
        LEFT,
        RIGHT
    }

    private IntakeMotor intake_motor_ ;

    private double power_ ; 

    public ZekeIntakePowerAction(ZekeIntakeSubsystem subsystem, IntakeMotor intake_motor, double power)  {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;
        intake_motor_ = intake_motor ;
        power_ = power ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;
        switch (intake_motor_) {
            case LEFT :
                subsystem_.setLeftCollectorPower(power_);
                break ;
            case RIGHT :
                subsystem_.setRightCollectorPower(power_);
                break ;
        }
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
