package frc.robot.gpm;

import org.xero1425.base.actions.Action;

import frc.robot.conveyor.ConveyorStopAction;
import frc.robot.intake.ZekeIntakeOffAction;
import frc.robot.shooter.SetShooterAction;

public class GPMStopCollectAction extends Action {
    private GPMSubsystem sub_;
    private ConveyorStopAction conveyor_stop_action_ ;
    private ZekeIntakeOffAction intake_stop_action_ ;
    private SetShooterAction shoot_ ;

    public GPMStopCollectAction(GPMSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;

        intake_stop_action_ = new ZekeIntakeOffAction(sub_.getIntake()) ;
    }

    public GPMStopCollectAction(GPMSubsystem sub, SetShooterAction act) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;

        intake_stop_action_ = new ZekeIntakeOffAction(sub_.getIntake()) ;
        shoot_ = act ;
    }

    @Override
    public void start() throws Exception {
        super.start();
        //  stop action -> intake
        sub_.getIntake().setAction(intake_stop_action_, true) ;

        //  stop action -> conveyor
        sub_.getConveyor().setAction(conveyor_stop_action_, true) ;

        if (shoot_ != null)
            sub_.getShooter().setAction(shoot_, true) ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        if (!sub_.getIntake().isBusy() && !sub_.getConveyor().isBusy()) {
            setDone() ;
        }

    }

    @Override
    public void cancel() {
        super.cancel();

        intake_stop_action_.cancel();

        if (shoot_ != null)
            shoot_.cancel() ;
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "GPMStopCollectAction";
    }

}
