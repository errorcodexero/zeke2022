package frc.robot.shooter;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motorsubsystem.MotorEncoderSubsystem;

public class ShooterSubsystem extends Subsystem {
    private MotorEncoderSubsystem wheelMotor1_;
    private MotorEncoderSubsystem wheelMotor2_;
    private MotorEncoderSubsystem hoodMotor_;
    public static final String SubsystemName = "shooter";

    public ShooterSubsystem(Subsystem parent) throws Exception {
        super(parent, SubsystemName);
        wheelMotor1_ = new MotorEncoderSubsystem(this, SubsystemName + "-w1", false);
        addChild(wheelMotor1_) ;

        wheelMotor2_ = new MotorEncoderSubsystem(this, SubsystemName + "-w2", false);
        addChild(wheelMotor2_);
        
        hoodMotor_ = new MotorEncoderSubsystem(this, SubsystemName + "-h", false);
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
}
