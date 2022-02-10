package frc.robot.conveyor;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motorsubsystem.PowerTestAction;
import org.xero1425.base.pneumatics.XeroDoubleSolenoid;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.zeke_color_sensor.ZekeColorSensor;
import frc.robot.zeke_color_sensor.ZekeColorSensor.CargoType;

public class ConveyorSubsystem extends Subsystem {
    private enum State {
        WAIT_INTAKE, 
        WAIT_COLOR,
        WAIT_EXIT1,
        WAIT_EXIT0,
        START_UPTAKE,
        WAIT_CHIMNEY1,
        WAIT_CHIMNEY0,
        WAIT_SHOOTER,
        WAIT_SHOOTER1,
        WAIT_SHOOTER0,
    }

    private static final DoubleSolenoid.Value ExitCloseState = DoubleSolenoid.Value.kForward ;
    private static final DoubleSolenoid.Value ExitOpenState = DoubleSolenoid.Value.kReverse ;
    public static final int MAX_BALLS = 2;
    private int ball_count_ ;                       // The number of balls stored in the conveyor
    private int ball_count_staged_ ;
    private boolean stop_collect_requested_;
    private CargoType[] ball_types_;
    private MotorController intake_motor_ ;         // The motor for the flash intake piece
    private double intake_motor_power_ ;
    private MotorController shooter_motor_ ;        // The motor for the shooter sidef of the conveyor
    private double shooter_motor_power_ ;
    private XeroDoubleSolenoid exit_ ;    
    private ZekeColorSensor color_sensor_;
    private static final int SENSOR_COUNT = 4;
    private static final int SENSOR_IDX_INTAKE = 0;
    private static final int SENSOR_IDX_EXIT = 1;
    private static final int SENSOR_IDX_CHIMNEY = 2;
    private static final int SENSOR_IDX_SHOOTER = 3;
    private State[] state_;
    private static final double POWER_OFF_ = 0;
    private double shooter_power_ = POWER_OFF_;


    private DigitalInput[] sensors_; // The array of ball detect sensors
    private boolean[] sensor_states_; // The states of ball detect sensors
    private boolean[] sensor_states_prev_; // The states of ball detect sensors
    
    public static final String SubsystemName = "conveyor";
    public static final String SensorLoggerName = "conveyor:sensors:messages";


    public ConveyorSubsystem(Subsystem parent, ZekeColorSensor color) throws Exception {
        super(parent, SubsystemName);
      
        color_sensor_ = color;
        exit_ = new XeroDoubleSolenoid(this, "exit") ;

        sensors_ = new DigitalInput[SENSOR_COUNT];
        sensor_states_ = new boolean[SENSOR_COUNT];
        sensor_states_prev_ = new boolean[SENSOR_COUNT];

        ball_count_ = 0 ;
        ball_count_staged_ = 0;
        ball_types_ = new CargoType [MAX_BALLS];
        state_ = new State[MAX_BALLS];
        for (int i = 0; i < MAX_BALLS; i++) {
            ball_types_ [i]= CargoType.None;
            state_[i] = State.WAIT_INTAKE;
        }
        intake_motor_ = getRobot().getMotorFactory().createMotor("intake", "subsystems:conveyor:hw:motors:intake");
        intake_motor_power_ = 0.0 ;
        shooter_motor_ = getRobot().getMotorFactory().createMotor("shooter", "subsystems:conveyor:hw:motors:shooter");
        shooter_motor_power_ = 0.0 ;
        stop_collect_requested_ = false;

        int num;
        int basech = (int) 'a';
        for (int i = 0; i < SENSOR_COUNT; i++) {
            String name = "hw:sensors:" + (char) (basech + i);
            num = getSettingsValue(name).getInteger() ;
            sensors_[i] = new DigitalInput(num);
            sensor_states_[i] = false;
            sensor_states_prev_[i] = false;
            String sname = Character.toString((char) ('A' + i));
            putDashboard(sname, Subsystem.DisplayType.Verbose, sensor_states_[i]);
        }
    }

    @Override
    public SettingsValue getProperty(String name) {
        SettingsValue v = null ;

        if (name.equals("ball-count")) {
            v = new SettingsValue(ball_count_staged_) ;
        }
        else if (name.equals("exit")) {
            v = new SettingsValue(exit_.get() == ExitOpenState) ;
        }
        else if (name.equals("horizontal")) {
            v = new SettingsValue(intake_motor_power_) ;
        }
        else if (name.equals("vertical")) {
            v = new SettingsValue(shooter_motor_power_) ;
        }

        return v ;
    }

    public boolean isFull() {
        return ball_count_ == MAX_BALLS ;
    }

    public boolean isEmpty() {
        return ball_count_ == 0 ;
    }

    public int getBallCount() {
        return ball_count_ ;
    }

    @Override
    public void postHWInit() {
        setDefaultAction(new ConveyorStopAction(this));
    }

    @Override
    public void computeMyState() throws Exception {
        for (int i = 0; i < SENSOR_COUNT; i++) {
            //
            // Get the sensor state
            //
            sensor_states_prev_[i] = sensor_states_[i];
            sensor_states_[i] = !sensors_[i].get();
        }
       CargoType cargoType =  color_sensor_.getCargoType(color_sensor_.getConveyorIndex());

       int loops = Math.min(ball_count_ + 1, MAX_BALLS);
       for (int i = 0; i < loops; i++) {
            State prevst = state_[i] ;

            switch (state_[i]) {
            case WAIT_INTAKE:
                if (sensor_states_[SENSOR_IDX_INTAKE]==true && sensor_states_[SENSOR_IDX_INTAKE] != sensor_states_prev_[SENSOR_IDX_INTAKE])
                {
                    ball_count_ ++;
                    state_[i] = State.WAIT_COLOR;
                }
                break;
            case  WAIT_COLOR:
                if (cargoType == CargoType.Opposite)
                {
                    exit_.set(ExitOpenState);
                    state_[i] = State.WAIT_EXIT1;
                }
                else if (cargoType == CargoType.Same)
                    state_[i] = State.START_UPTAKE;
                break;
            case  WAIT_EXIT1:
                if (sensor_states_[SENSOR_IDX_EXIT]==true)
                    state_[i] = State.WAIT_EXIT0;
                break;
            case  WAIT_EXIT0:
                if (sensor_states_[SENSOR_IDX_EXIT]==false)
                {
                    exit_.set(ExitCloseState);
                    state_[i] = State.WAIT_INTAKE; 
                    removeBall();
                }
                break;
            case  START_UPTAKE:
                if (i == 0)
                {
                    shooter_motor_power_= shooter_power_;
                    shooter_motor_.set(shooter_power_);
                    state_[i] = State.WAIT_CHIMNEY1;
                }
                else
                {
                    ball_count_staged_ = 2;
                    intake_motor_power_= POWER_OFF_;
                    intake_motor_.set(POWER_OFF_);
                }
                break;
            case  WAIT_CHIMNEY1:
                if (sensor_states_[SENSOR_IDX_CHIMNEY]==true)
                    state_[i] = State.WAIT_CHIMNEY0;
                break;
            case  WAIT_CHIMNEY0:
                if (sensor_states_[SENSOR_IDX_CHIMNEY]==false)
                {
                    ball_count_staged_ = 1;
                    shooter_motor_power_= POWER_OFF_;
                    shooter_motor_.set(POWER_OFF_);
                    state_[i] = State.WAIT_SHOOTER;
                }
                break;
           /* case  WAIT_SHOOTER:
                shooter_motor_.set(POWER_OFF_); 
                break; */
            case WAIT_SHOOTER1:
                if (sensor_states_[SENSOR_IDX_SHOOTER]==true)
                    state_[i] = State.WAIT_SHOOTER0;
                break;
            case WAIT_SHOOTER0:
                if (sensor_states_[SENSOR_IDX_SHOOTER]==false)
                {
                    ball_count_staged_--;
                    shooter_motor_power_= POWER_OFF_;
                    shooter_motor_.set(POWER_OFF_);
                    state_[i] = State.WAIT_INTAKE;
                    removeBall();
                }
                    break;
            }

            if (prevst != state_[i]) {
                MessageLogger logger = getRobot().getMessageLogger() ;
                logger.startMessage(MessageType.Debug, getLoggerID()) ;
                logger.add("ConveyorSubsystem:") ;
                logger.add("loop", i) ;
                logger.add(" ").add(prevst.toString()).add(" --> ").add(state_[i].toString()) ;
                logger.endMessage();
            }
        }

        if (stop_collect_requested_ && (
            (ball_count_ == 0) ||
            ((ball_count_ == 1) && (state_[0] == State.WAIT_SHOOTER)) ||
            ((ball_count_ == 2) && (state_[0] == State.WAIT_SHOOTER) && (state_[1] == State.START_UPTAKE))))
        {
            setMotorsPower(POWER_OFF_, POWER_OFF_);

            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getLoggerID()) ;
            logger.add("ConveyorSubsystem: shutting down as requested.") ;
            logger.endMessage();
        }

        putDashboard("ballcount", DisplayType.Always, ball_count_);
    } 

    @Override
    public void run() throws Exception {
        super.run() ;
    }

    
    protected void setMotorsPower(double intake, double shooter) {
        try {
            MessageLogger logger = getRobot().getMessageLogger();
            logger.startMessage(MessageType.Debug, getLoggerID());
            logger.add("Conveyor:").add("intake_power", intake) ;
            logger.add(" shooter_power", shooter) ;
            logger.endMessage();

            stop_collect_requested_ = false;
            intake_motor_.set(intake) ;
            intake_motor_power_ = intake ;
            shooter_power_ = shooter ;
        }
        catch(Exception ex) {
        }
    }

    protected void setStopCollect()
    {
        MessageLogger logger = getRobot().getMessageLogger();
        logger.startMessage(MessageType.Debug, getLoggerID());
        logger.add("Conveyor: stop collect requested.") ;
        logger.endMessage();

        stop_collect_requested_ = true;
    }

    protected void setPreloadedBall() 
    {
        ball_types_ [0] =  CargoType.Same;
        state_ [0] = State.START_UPTAKE;
        ball_count_ = 1;
    }
        
    protected void setShootMode(double shooter) {
        try {
            MessageLogger logger = getRobot().getMessageLogger();
            logger.startMessage(MessageType.Debug, getLoggerID());
            logger.add("Conveyor:");
            logger.add(" shooter_power", shooter) ;
            logger.endMessage();

            shooter_motor_power_ = shooter;
            shooter_motor_.set(shooter) ;
            state_[0] = State.WAIT_SHOOTER1;
        }
        catch(Exception ex) {
        }
    }  
    private void removeBall() 
    {
        for(int i = 1; i < ball_count_; i ++) 
        {
           ball_types_ [i-1] = ball_types_ [i];
           state_ [i-1] = state_[i];
        }
        ball_count_ --;
        ball_types_ [ball_count_] =  CargoType.None;
        state_ [ball_count_] = State.WAIT_INTAKE;
        
    }
 } ;
