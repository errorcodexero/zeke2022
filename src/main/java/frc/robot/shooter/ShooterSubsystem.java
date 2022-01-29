package frc.robot.shooter;

import com.fasterxml.jackson.core.sym.Name;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.base.motorsubsystem.MotorEncoderVelocityAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class ShooterSubsystem extends Subsystem {
    private MotorEncoderSubsystem wheelMotor1_;
    private MotorEncoderSubsystem wheelMotor2_;
    private MotorEncoderSubsystem hoodMotor_;
    public static final String SubsystemName = "shooter";

    public ShooterSubsystem(Subsystem parent) throws Exception {
        super(parent, SubsystemName);
        wheelMotor1_ = new MotorEncoderSubsystem(this, SubsystemName + "-w1", false);
        wheelMotor2_ = new MotorEncoderSubsystem(this, SubsystemName + "-w2", false);
        hoodMotor_ = new MotorEncoderSubsystem(this, SubsystemName + "-h", false);
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
  
    public void setWheelVelocity(double v1, double v2) throws BadMotorRequestException, MotorRequestFailedException, MissingParameterException, BadParameterTypeException{
        wheelMotor1_ = getWheelMotor1();
        wheelMotor2_ = getWheelMotor2();
        MotorEncoderVelocityAction a1 = new MotorEncoderVelocityAction(wheelMotor1_, "wheel1Velocity", v1);
        MotorEncoderVelocityAction a2 = new MotorEncoderVelocityAction(wheelMotor2_, "wheel2Velocity", v2);
        wheelMotor1_.setAction(a1);
        wheelMotor2_.setAction(a2);
    }

    public void setHoodAngle(double angle) throws BadMotorRequestException, MotorRequestFailedException, MissingParameterException, BadParameterTypeException{
        hoodMotor_ = getHoodMotor();
        MotorEncoderVelocityAction a1 = new MotorEncoderVelocityAction(hoodMotor_, "hoodPosition", angle);
        hoodMotor_.setAction(a1);
    }
}
