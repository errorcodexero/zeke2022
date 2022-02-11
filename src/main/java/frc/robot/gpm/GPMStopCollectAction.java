package frc.robot.gpm;

import org.xero1425.base.actions.Action;

import frc.robot.conveyor.ConveyorStopAction;
import frc.robot.intake.ZekeIntakeOffAction;

public class GPMStopCollectAction extends Action {
    private GPMSubsystem sub_;
    private ConveyorStopAction conveyor_stop_action_ ;
    private ZekeIntakeOffAction intake_stop_action_ ;

    public GPMStopCollectAction(GPMSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;

        conveyor_stop_action_ = new ConveyorStopAction(sub_.getConveyor()) ; 

        intake_stop_action_ = new ZekeIntakeOffAction(sub_.getIntake()) ;
    }

    @Override
    public void start() throws Exception {
        super.start();
        //  stop action -> intake
        sub_.getIntake().setAction(conveyor_stop_action_, true) ;

        //  stop action -> conveyor
        sub_.getConveyor().setAction(conveyor_stop_action_, true) ;
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

        conveyor_stop_action_.cancel() ;
        intake_stop_action_.cancel();
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "GPMStopCollectAction";
    }

}