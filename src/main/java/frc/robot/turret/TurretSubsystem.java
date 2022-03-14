package frc.robot.turret;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motorsubsystem.MotorEncoderSubsystem;

public class TurretSubsystem extends MotorEncoderSubsystem {
    
    private double min_safe_angle_ ;
    private double max_safe_angle_ ;
    private boolean is_ready_to_fire_ ;
    
    public final static String SubsystemName = "turret" ;

    public TurretSubsystem(Subsystem parent) throws Exception {
        super(parent, SubsystemName, false) ;

        min_safe_angle_ = getSettingsValue("min").getDouble() ;
        max_safe_angle_ = getSettingsValue("max").getDouble() ;
        is_ready_to_fire_ = false ;
    }

    public double getMinSafeAngle() {
        return min_safe_angle_ ;
    }

    public double getMaxSafeAngle() {
        return max_safe_angle_ ;
    }

    public double limitAngleToSafeRange(double angle) {
        if (angle < getMinSafeAngle()) {
            return getMinSafeAngle() ;
        }
        else if (angle > getMaxSafeAngle()) {
            return getMaxSafeAngle() ;
        } else {
            return angle;
        }
    }

    public boolean isReadyToFire() {
        return is_ready_to_fire_ ;
    }

    public void setReadyToFire(boolean b) {
        is_ready_to_fire_ = b ;
    }

    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();

        putDashboard("turret", DisplayType.Verbose, getPosition());
    }

    @Override
    protected double limitPower(double p) {
        if (p < 0 && getPosition() < getMinSafeAngle())
            p = 0 ;
        else if (p > 0 && getPosition() > getMaxSafeAngle())
            p = 0 ;

        return p ;
    }
}

