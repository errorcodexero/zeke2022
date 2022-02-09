package frc.robot.shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motorsubsystem.MotorEncoderGotoAction;
import org.xero1425.base.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.base.motorsubsystem.MotorEncoderTrackPositionAction;
import org.xero1425.base.motorsubsystem.MotorEncoderVelocityAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class SetShooterAction extends Action {
    private ShooterSubsystem sub_;
    private MotorEncoderVelocityAction w1_action_;
    private MotorEncoderVelocityAction w2_action_;
    private MotorEncoderTrackPositionAction hood_action_;

    public SetShooterAction(ShooterSubsystem sub, double w1, double w2, double hood) throws Exception
    {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub;
        w1_action_ = new MotorEncoderVelocityAction(sub.getWheelMotor1(), "w1", w1);
        w2_action_ = new MotorEncoderVelocityAction(sub.getWheelMotor2(), "w2", w2);
        hood_action_ = new MotorEncoderTrackPositionAction(sub.getHoodMotor(), "hood", hood);
    }

    public void update(double w1, double w2, double hood) throws BadMotorRequestException, MotorRequestFailedException{
        w1_action_.setTarget(w1);
        w2_action_.setTarget(w2);
        hood_action_.setTarget(hood);
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
    }

    @Override
    public String toString(int indent){
        return spaces(indent) + "SetShooterAction";
    }



}
