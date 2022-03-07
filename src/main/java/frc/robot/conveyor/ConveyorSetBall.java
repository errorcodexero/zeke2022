package frc.robot.conveyor;

import org.xero1425.base.actions.Action;

public class ConveyorSetBall extends Action {
    private ConveyorSubsystem sub_ ;

    public ConveyorSetBall(ConveyorSubsystem sub) {
        super(sub.getRobot().getMessageLogger()) ;

        sub_ = sub ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;
        sub_.setPreloadedBall();
        setDone() ;
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "ConveyorSetBall" ;
    }
}
