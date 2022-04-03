package frc.robot.shooter;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.misc.SettingsValue;

public class ShooterSubsystem extends Subsystem {
    private MotorEncoderSubsystem wheelMotor1_;
    private MotorEncoderSubsystem wheelMotor2_;
    private MotorEncoderSubsystem hoodMotor_;
    public static final String SubsystemName = "shooter";

    public ShooterSubsystem(Subsystem parent) throws Exception {
        super(parent, SubsystemName);

        double ticks_per_rev = 42 ;
        double seconds_per_minute = 60 ;
        double motor_to_shooter_gear_ratio  = 2 / 1 ;
        double factor =  seconds_per_minute / ticks_per_rev * motor_to_shooter_gear_ratio ;

        wheelMotor1_ = new MotorEncoderSubsystem(this, SubsystemName + "-w1", false, 8);
        addChild(wheelMotor1_) ;
        wheelMotor1_.setVelocityConversion(factor);

        wheelMotor2_ = new MotorEncoderSubsystem(this, SubsystemName + "-w2", false, 8);
        addChild(wheelMotor2_);
        wheelMotor2_.setVelocityConversion(factor);
        
        hoodMotor_ = new HoodMotorSubsystem(this) ;
        addChild(hoodMotor_) ;
    }

    public MotorEncoderSubsystem getWheelMotor1(){
        return wheelMotor1_;
    }
    public MotorEncoderSubsystem getWheelMotor2(){
        return wheelMotor2_;
    }
    public MotorEncoderSubsystem getHoodMotor(){
        return hoodMotor_;
    }

    @Override
    public SettingsValue getProperty(String name) {
        SettingsValue v = null ;

        if (name.equals("wheel1-velocity")) {
            v = new SettingsValue(wheelMotor1_.getVelocity()) ;
        }
        else if (name.equals("wheel2-velocity")) {
            v = new SettingsValue(wheelMotor2_.getVelocity()) ;
        }
        else if (name.equals("hood-angle")) {
            v = new SettingsValue(hoodMotor_.getPosition()) ;
        }

        return v ;
    }
}
