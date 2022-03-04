package frc.robot.shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motorsubsystem.MotorEncoderTrackPositionAction;
import org.xero1425.base.motorsubsystem.MotorEncoderVelocityAction;

public class SetShooterAction extends Action {
    private ShooterSubsystem sub_;
    private MotorEncoderVelocityAction w1_action_;
    private MotorEncoderVelocityAction w2_action_;
    private MotorEncoderTrackPositionAction hood_action_;

    private static int plot_number_ = 0 ;

    private int plot_id_ ;
    private double plot_start_ ;
    private Double [] plot_data_ ;

    // The columns to plot
    private static String [] columns_ = { "time", "ltarget(rpm)", "lactual(rpm)", "rtarget(rpm)", "ractual(rpm)", "htarget(degrees)", "hactual(degrees)" } ;

    public SetShooterAction(ShooterSubsystem sub, double w1, double w2, double hood) throws Exception
    {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub;
        w1_action_ = new MotorEncoderVelocityAction(sub.getWheelMotor1(), "w1", w1);
        w2_action_ = new MotorEncoderVelocityAction(sub.getWheelMotor2(), "w2", w2);
        hood_action_ = new MotorEncoderTrackPositionAction(sub.getHoodMotor(), "hood", hood);

        plot_id_ = -1 ;
        plot_data_ = new Double[7] ;
    }

    public void update(double w1, double w2, double hood) throws BadMotorRequestException, MotorRequestFailedException{
        w1_action_.setTarget(w1);
        w2_action_.setTarget(w2);
        hood_action_.setTarget(hood);
    }

    public void startPlot() {
        plot_id_ = sub_.initPlot("SetShooterAction-" + plot_number_++) ;
        sub_.startPlot(plot_id_, columns_);
        plot_start_ = sub_.getRobot().getTime() ;
    }

    public void stopPlot() {
        sub_.endPlot(plot_id_) ;
        plot_id_ = -1 ;
    }

    @Override
    public void start() throws Exception{
        super.start();
        sub_.getWheelMotor1().setAction(w1_action_, true);
        sub_.getWheelMotor2().setAction(w2_action_, true);
        sub_.getHoodMotor().setAction(hood_action_, true);
    }

    @Override
    public void run() throws Exception{
        super.run();

        plot_data_[0] = sub_.getRobot().getTime() - plot_start_ ;
        plot_data_[1] = w1_action_.getTarget() ;
        plot_data_[2] = sub_.getWheelMotor1().getVelocity() ;
        plot_data_[3] = w1_action_.getTarget() ;
        plot_data_[4] = sub_.getWheelMotor2().getVelocity() ;
        plot_data_[5] = hood_action_.getTarget() ;
        plot_data_[6] = sub_.getHoodMotor().getPosition() ;
        sub_.addPlotData(plot_id_, plot_data_) ;
    }

    @Override
    public String toString(int indent){
        return spaces(indent) + "SetShooterAction";
    }

    @Override
    public void cancel() {
        sub_.getWheelMotor1().cancelAction();
        sub_.getWheelMotor2().cancelAction();
    }
}
