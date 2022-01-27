package frc.robot.gpm.intake;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;


public class ZekeIntakeSubsystem extends Subsystem {
    public static final String SubsystemName = "intake";

    private MotorController collector_a_;
    private MotorController collector_b_;
    private Solenoid solenoid;


    public ZekeIntakeSubsystem(Subsystem parent) throws Exception {
        super(parent, SubsystemName);        

        // Motor 2, explicitly create it
        collector_a_ = getRobot().getMotorFactory().createMotor("intake-collector-a",
                "subsystems:intake:hw:collector:motor-a");
        collector_b_ = getRobot().getMotorFactory().createMotor("intake-collecor-b", "subsystems:intake:hw:collector:motor-b");
        
        

        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        
    }

    public void setCollectorPower(double pa, double pb) throws BadMotorRequestException, MotorRequestFailedException {
        collector_a_.set(pa);
        collector_b_.set(pb);
    }

    public void setSolenoidPower(boolean v) {
        solenoid.set(v);
    }


    @Override
    public void postHWInit() {
        
        setSolenoidPower(false);
       
    }
    
}
