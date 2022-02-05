package frc.robot.gpm;

import org.xero1425.base.actions.Action;

public class GPMFireAction extends Action {
    private GPMSubsystem sub_;
    
    public GPMFireAction(GPMSubsystem sub) 
            throws Exception {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub ;
    }

    @Override
    public void start() throws Exception {
        super.start();   
    }

    @Override
    public void run() throws Exception {
        super.run();
    }

    @Override
    public void cancel() {
        super.cancel();
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "GPMFireAction";
    }

}