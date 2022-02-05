package frc.robot.conveyor;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.pneumatics.XeroDoubleSolenoid;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.zeke_color_sensor.ZekeColorSensor;

public class ConveyorSubsystem extends Subsystem {
    private static final DoubleSolenoid.Value ExitCloseState = DoubleSolenoid.Value.kForward ;
    private static final DoubleSolenoid.Value ExitOpenState = DoubleSolenoid.Value.kReverse ;
    private int ball_count_ ;                       // The number of balls stored in the conveyor

    private MotorController intake_motor_ ;         // The motor for the flash intake piece
    private MotorController shooter_motor_ ;        // The motor for the shooter sidef of the conveyor
    private XeroDoubleSolenoid Exit ;    
    private ZekeColorSensor color_sensor_;
    private static final int SENSOR_COUNT = 4;
    public static final int SENSOR_IDX_INTAKE = 0;
    public static final int SENSOR_IDX_EXIT = 1;
    public static final int SENSOR_IDX_CHIMNEY = 2;
    public static final int SENSOR_IDX_SHOOTER = 3;

    private DigitalInput[] sensors_; // The array of ball detect sensors
    private boolean[] sensor_states_; // The states of ball detect sensors
    private boolean[] prev_sensor_states_; // The states of ball detect sensors last robot loop
    
    public static final String SubsystemName = "conveyor";
    public static final String SensorLoggerName = "conveyor:sensors:messages";
    public static final int MAX_BALLS = 2;


    public ConveyorSubsystem(Subsystem parent, ZekeColorSensor color) throws Exception {
        super(parent, SubsystemName);
      
      color_sensor_ = color;
        Exit = new XeroDoubleSolenoid(this, "Exit") ;

        sensors_ = new DigitalInput[SENSOR_COUNT];
        sensor_states_ = new boolean[SENSOR_COUNT];
        prev_sensor_states_ = new boolean[SENSOR_COUNT];

        ball_count_ = 0 ;

        intake_motor_ = getRobot().getMotorFactory().createMotor("intake", "subsystems:conveyor:hw:motors:intake");
        shooter_motor_ = getRobot().getMotorFactory().createMotor("shooter", "subsystems:conveyor:hw:motors:shooter");

        int num;
        int basech = (int) 'a';
        for (int i = 0; i < SENSOR_COUNT; i++) {
            String name = "hw:sensors:" + (char) (basech + i);
            num = getSettingsValue(name).getInteger() ;
            sensors_[i] = new DigitalInput(num);
            sensor_states_[i] = false;
            prev_sensor_states_[i] = false;
            String sname = Character.toString((char) ('A' + i));
            putDashboard(sname, Subsystem.DisplayType.Verbose, sensor_states_[i]);
        }
    }

    public SettingsValue getProperty(String name) {
        SettingsValue v = null ;
        
        if (name.equals("ballcount")) {
            v = new SettingsValue(ball_count_) ;
        }
        return v ;
    }

    public boolean isFull() {
        return ball_count_ == MAX_BALLS ;
    }

    public boolean isEmpty() {
        return ball_count_ == 0 ;
    }


    @Override
    public void run() throws Exception {
        super.run() ;
        for (int i = 0; i < SENSOR_COUNT; i++) {
                //
                // Get the sensor state
                //
                prev_sensor_states_[i] = sensor_states_[i];
                sensor_states_[i] = !sensors_[i].get();
            }
        }


    public void setStagedForCollect(boolean staged) {

        MessageLogger logger = getRobot().getMessageLogger();
        logger.startMessage(MessageType.Debug, getLoggerID());
        logger.endMessage();
    }

    @Override
    public void postHWInit() {
        setDefaultAction(new ConveyorStopAction(this));
    }

    @Override
    public void computeMyState() throws Exception {
        putDashboard("ballcount", DisplayType.Always, ball_count_);
    }

    public int getBallCount() {
        return ball_count_ ;
    }

    protected void setBallCount(int n) {
        ball_count_ = n ;
    }

    protected void incrementBallCount() {
        ball_count_++ ;

        MessageLogger logger = getRobot().getMessageLogger();
        logger.startMessage(MessageType.Debug, getLoggerID());
        logger.add("Conveyor:").add("ballcount", ball_count_);
        logger.endMessage();
    }

    protected void decrementBallCount() {
        ball_count_-- ;

        MessageLogger logger = getRobot().getMessageLogger();
        logger.startMessage(MessageType.Debug, getLoggerID());
        logger.add("Conveyor:").add("ballcount", ball_count_);
        logger.endMessage();
    }    

    protected void setMotorsPower(double intake, double shooter) {
        try {
            MessageLogger logger = getRobot().getMessageLogger();
            logger.startMessage(MessageType.Debug, getLoggerID());
            logger.add("Conveyor:").add("intake_power", intake) ;
            logger.add(" shooter_power", shooter) ;
            logger.endMessage();

            intake_motor_.set(intake) ;
            shooter_motor_.set(shooter) ;
        }
        catch(Exception ex) {
        }
    }  
} ;