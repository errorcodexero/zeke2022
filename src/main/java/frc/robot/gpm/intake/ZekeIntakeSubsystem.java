package frc.robot.gpm.intake;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.wpilibj.Solenoid;


public class ZekeIntakeSubsystem extends Subsystem {
    public static final String SubsystemName = "intake";

    private MotorController collector_a_;
    private MotorController collector_b_;
    private Solenoid solenoid;
    private BallColors myColor;

    public enum BallColors {
        SAME,
        OPPOSITE,
        NOTHING
    }


    public ZekeIntakeSubsystem(Subsystem parent) throws Exception {
        super(parent, SubsystemName);        

        // Motor 2, explicitly create it
        collector_a_ = getRobot().getMotorFactory().createMotor("intake-collector-a",
                "subsystems:intake:hw:collector:motor-a");
        collector_b_ = getRobot().getMotorFactory().createMotor("intake-collecor-b", "subsystems:intake:hw:collector:motor-b");
          
        solenoid = new Solenoid(getRobot().getPneumaticsType(), 1);
        
    }

    public void setCollectorPower(double pa, double pb) throws BadMotorRequestException, MotorRequestFailedException {
        collector_a_.set(pa);
        collector_b_.set(pb);
    }

    public void setSolenoidPower(boolean v) {
        solenoid.set(v);
    }

    public BallColors getLeftBallColor() {
        return BallColors.SAME;
    }
    public BallColors getRightBallColor() {
        return BallColors.OPPOSITE;
    }
    public boolean isIntakeBlocked() {
        return getLeftBallColor() != BallColors.NOTHING && getRightBallColor() != BallColors.NOTHING;
    }



    public void clearIntake() throws BadParameterTypeException, MissingParameterException, BadMotorRequestException, MotorRequestFailedException, InterruptedException {
        double collector_motor_a_power_ = getSettingsValue("hw:collector:motor-a:power").getDouble();
        double collector_motor_b_power_ = getSettingsValue("hw:collector:motor-b:power").getDouble();

        if (!isIntakeBlocked()) {
            setCollectorPower(collector_motor_a_power_, collector_motor_b_power_);
            return;
        }

        if (getRightBallColor() == BallColors.OPPOSITE && getLeftBallColor() == BallColors.OPPOSITE) {
            setCollectorPower(-collector_motor_a_power_, -collector_motor_b_power_);
        }

        if (getLeftBallColor() == BallColors.SAME && getRightBallColor() == BallColors.OPPOSITE) {
            setCollectorPower(collector_motor_a_power_, -collector_motor_b_power_);
        }
        
        if (getLeftBallColor() == BallColors.OPPOSITE && getRightBallColor() == BallColors.SAME) {
            setCollectorPower(-collector_motor_a_power_, collector_motor_b_power_);
        }
        if (getLeftBallColor() == BallColors.OPPOSITE && getRightBallColor() == BallColors.NOTHING) {
            setCollectorPower(-collector_motor_a_power_, collector_motor_b_power_);
        }
        
        if (getLeftBallColor() == BallColors.NOTHING && getRightBallColor() == BallColors.OPPOSITE) {
            setCollectorPower(collector_motor_a_power_, -collector_motor_b_power_);
        }

        if (getLeftBallColor() == BallColors.SAME && getRightBallColor() == BallColors.SAME) {
            setCollectorPower(-0.1, collector_motor_b_power_);
            getRobot().wait(15L);
            setCollectorPower(collector_motor_a_power_, collector_motor_b_power_);
        }
    }

    @Override
    public void postHWInit() {
        
        setSolenoidPower(false);
       
    }
    
}
