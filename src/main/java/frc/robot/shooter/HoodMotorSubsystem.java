package frc.robot.shooter;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motorsubsystem.MotorEncoderSubsystem;

public class HoodMotorSubsystem extends MotorEncoderSubsystem {
    private double max_pos_ ;
    private double min_pos_ ;

    public HoodMotorSubsystem(Subsystem parent) throws Exception {
        super(parent, "shooter-h", false) ;

        max_pos_ = getSettingsValue("maxpos").getDouble() ;
        min_pos_ = getSettingsValue("minpos").getDouble() ;
    }

    @Override
    protected double limitPower(double p) {
        if (getPosition() <= min_pos_ && p < 0.0)
            p = 0.0 ;
        else if (getPosition() >= max_pos_ && p > 0.0)
            p = 0.0 ;

        return p ;
    }
}
