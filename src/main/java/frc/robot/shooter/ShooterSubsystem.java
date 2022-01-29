package frc.robot.shooter;

import com.fasterxml.jackson.core.sym.Name;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motorsubsystem.MotorEncoderSubsystem;

public class ShooterSubsystem extends Subsystem {
    private MotorController wheelMotor1_;
    private MotorController wheelMotor2_;
    private MotorController hoodMotor_;
    public static final String SubsystemName = "shooter";

    public ShooterSubsystem(Subsystem parent) {
        super(parent, SubsystemName);
        wheelMotor1_ = getRobot().getMotorFactory().createMotor("wheelMotor1_", "subsystems:shooter:hw:motors:wheelMotor1_");
        wheelMotor2_ = getRobot().getMotorFactory().createMotor("wheelMotor2_", "subsystems:shooter:hw:motors:wheelMotor2_");
        hoodMotor_ = getRobot().getMotorFactory().createMotor("hoodMotor_", "subsystems:shooter:hw:motors:hoodMotor_");
    }

    public MotorController getWheelMotor1(){
        return wheelMotor1_;
    }
    public MotorController getWheelMotor2(){
        return wheelMotor2_;
    }
    public MotorController getHoodMotor(){
        return hoodMotor_;
    }

}
