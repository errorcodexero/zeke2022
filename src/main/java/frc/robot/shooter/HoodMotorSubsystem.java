package frc.robot.shooter;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motorsubsystem.MotorEncoderSubsystem;

public class HoodMotorSubsystem extends MotorEncoderSubsystem {

    public HoodMotorSubsystem(Subsystem parent) throws Exception {
        super(parent, "shooter-h", false) ;
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
