package frc.robot.shooter;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

public class HoodMotorSubsystem extends MotorEncoderSubsystem {
    private double max_pos_ ;
    private double min_pos_ ;

    public HoodMotorSubsystem(Subsystem parent) throws Exception {
        super(parent, "shooter-h", false) ;

        max_pos_ = getSettingsValue("maxpos").getDouble() ;
        min_pos_ = getSettingsValue("minpos").getDouble() ;
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
}
