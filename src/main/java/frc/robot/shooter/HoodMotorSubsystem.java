package frc.robot.shooter;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motorsubsystem.MotorEncoderSubsystem;

public class HoodMotorSubsystem extends MotorEncoderSubsystem {
    private double max_value_ ;
    private double min_value_ ;

    public HoodMotorSubsystem(Subsystem parent) throws Exception {
        super(parent, "shooter-h", false) ;

        max_value_ = 22.0 ;
        min_value_ = 1.0 ;
    }

    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();

        putDashboard("hood", DisplayType.Verbose, getPosition());
    }

    @Override
    public void run() throws Exception {
        super.run() ;
    }

    @Override
    public double limitPower(double p) {
        if (getPosition() >= max_value_ && p > 0.0) {
            p = 0.0 ;
        } if (getPosition() <= min_value_ && p < 0.0) {
            p = 0.0 ;
        }

        return p ;
    }
}
