package frc.robot.intake;

import org.xero1425.base.actions.Action;

public class ZekeIntakeArmAction extends Action {
    private ZekeIntakeSubsystem subsystem_;

    public enum ArmPos {
        RETRACT,
        DEPLOY
    }

    private ArmPos arm_pos_ ;

    public ZekeIntakeArmAction(ZekeIntakeSubsystem subsystem, ArmPos arm_pos)  {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem ;
        arm_pos_ = arm_pos ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;
        switch (arm_pos_) {
            case RETRACT:
                subsystem_.retractIntake();
                break ;
            case DEPLOY:
                subsystem_.deployIntake();
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
        return prefix(indent) + "IntakeArmAction" + arm_pos_.toString() ;
    }
    
}
