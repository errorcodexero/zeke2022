package org.xero1425.base.motorsubsystem;

import org.xero1425.misc.PIDCtrl;

public class MotorEncoderTrackPositionAction extends MotorAction {
    
    // The target position
    private double target_ ;

    // The PID controller to follow the plan
    private PIDCtrl ctrl_ ;

    // The error between the subsystem position and the target
    private double error_ ;

    // The time the last loop was run
    private double last_time_ ;

    /// \brief Create the action
    /// \param sub the MotorEncoderSubsystem subsystem for the action    
    /// \param target the target position
    /// \param addhold if true, add a hold action when the goto action is complete
    public MotorEncoderTrackPositionAction(MotorEncoderSubsystem sub, double target) throws Exception {
        super(sub) ;

        if (!(sub instanceof MotorEncoderSubsystem))
            throw new Exception("This subsystem is not a MotorEncoderSubsystem") ;
                    
        target_ = target ;
    }

    public double getError() {
        return error_ ;
    }

    /// \brief Start the action, computing the plan using the trapezoidal profile.  This method also
    /// initializes the PID controller based on whether or not the motion us "up" or "down".
    public void start() throws Exception {
        super.start() ;
    }

    public void setTarget(double t) {
        MotorEncoderSubsystem sub = (MotorEncoderSubsystem)getSubsystem() ;

        target_ = t ;
        error_ = Math.abs(target_ - sub.getPosition()) ;
    }

    /// \brief Called once per robot loop to adjust the motor power to follow the motion plan
    // given by the TrapezoidalProfile class.
    public void run() throws Exception {
        super.run() ;

        MotorEncoderSubsystem sub = (MotorEncoderSubsystem)getSubsystem() ;
        double t = sub.getRobot().getTime() ;

        double out = ctrl_.getOutput(target_, sub.getPosition(), t - last_time_) ;
        sub.setPower(out) ;
        last_time_ = t ;

        error_ = Math.abs(target_ - sub.getPosition()) ;
    }

    /// \brief Cancel the action and set the motor power to zero
    public void cancel() {
        super.cancel() ;

        getSubsystem().setPower(0.0) ;
    }

    /// \brief Returns a human readable string describing the action
    /// \returns a human readable string describing the action
    public String toString(int indent) {
        return prefix(indent) + "MotorEncoderGotoAction," + getSubsystem().getName() + "," + Double.toString(target_) ;
    }
}
