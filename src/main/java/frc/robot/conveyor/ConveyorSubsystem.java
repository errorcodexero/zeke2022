package frc.robot.conveyor;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.pneumatics.XeroSolenoid;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.zeke_color_sensor.ZekeColorSensor;
import frc.robot.zeke_color_sensor.ZekeColorSensor.CargoType;

public class ConveyorSubsystem extends Subsystem {
    // States in the state machine
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


    // Constanst

    // The maximum number of balls present
    public static final int MAX_BALLS = 2;

    // The total number of sensors
    private static final int SENSOR_COUNT = 4;

    // The indexes of the various sensors
    private static final int SENSOR_IDX_INTAKE = 0;
    private static final int SENSOR_IDX_EXIT = 1;
    private static final int SENSOR_IDX_CHIMNEY = 2;
    private static final int SENSOR_IDX_SHOOTER = 3;

    // The state of the exit XeroSolenoid when the exit portal is closed
    private static final boolean ExitCloseState = false ;

    // The state of the exit XeroSolenoid when the exit portal is open
    private static final boolean ExitOpenState = true ;

    private static final String[] SensorNames = { "intake", "exit", "chimney", "shooter"} ;

    private boolean bypass_ ;
    private int ball_count_ ;                       // The number of balls stored in the conveyor
    private int ball_count_staged_ ;
    private boolean stop_collect_requested_;
    private CargoType[] ball_types_;

    private MotorController intake_motor_ ;         // The motor for the flash intake piece
    private double intake_motor_power_ ;
    private double intake_motor_on_ ;

    private MotorController shooter_motor_ ;        // The motor for the shooter sidef of the conveyor
    private double shooter_motor_power_ ;
    private double shooter_motor_on_ ;

    private XeroSolenoid exit_ ;    
    private ZekeColorSensor color_sensor_;

    private State[] state_;

    private static final double POWER_OFF_ = 0;


    private DigitalInput[] sensors_; // The array of ball detect sensors
    private boolean[] sensor_states_; // The states of ball detect sensors
    private boolean[] sensor_states_prev_; // The states of ball detect sensors
    
    public static final String SubsystemName = "conveyor";
    public static final String SensorLoggerName = "conveyor:sensors:messages";


    public ConveyorSubsystem(Subsystem parent, ZekeColorSensor color) throws Exception {
        super(parent, SubsystemName);
      
        color_sensor_ = color;
        exit_ = new XeroSolenoid(this, "exit") ;

        bypass_ = false ;

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
        intake_motor_on_ = getSettingsValue("power:intake").getDouble() ;

        shooter_motor_ = getRobot().getMotorFactory().createMotor("shooter", "subsystems:conveyor:hw:motors:shooter");
        shooter_motor_power_ = 0.0 ;
        shooter_motor_on_ = getSettingsValue("power:shooter").getDouble() ;

        stop_collect_requested_ = false;

        int num;
        String name = null ;
        for (int i = 0; i < SENSOR_COUNT; i++) {
            name = SensorNames[i] ;
            num = getSettingsValue("hw:sensors:" + name).getInteger() ;
            sensors_[i] = new DigitalInput(num);
            sensor_states_[i] = false;
            sensor_states_prev_[i] = false;
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

    @Override
    public void postHWInit() {
        setDefaultAction(new ConveyorStopAction(this));
    }

    @Override
    public void computeMyState() throws Exception {

        CargoType cargoType =  color_sensor_.getCargoType(color_sensor_.getConveyorIndex());

        MessageLogger logger = getRobot().getMessageLogger() ;
        for (int i = 0; i < SENSOR_COUNT; i++) {
            //
            // Get the sensor state
            //
            sensor_states_prev_[i] = sensor_states_[i];
            sensor_states_[i] = !sensors_[i].get();

            String sname = SensorNames[i] ;
            putDashboard(sname, Subsystem.DisplayType.Always, sensor_states_[i]);
        }

        if (!bypass_) {
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
                        removeBall(i);
                    }
                    break;
                case  START_UPTAKE:
                    if (i == 0)
                    {
                        setShooterMotor(shooter_motor_on_);
                        state_[i] = State.WAIT_CHIMNEY1;
                    }
                    else
                    {
                        ball_count_staged_ = 2;
                        setIntakeMotor(POWER_OFF_);
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
                        setShooterMotor(POWER_OFF_);
                        state_[i] = State.WAIT_SHOOTER;
                    }
                    break;
                case  WAIT_SHOOTER:
                    if (ball_count_ == 2 && i == 1) {
                        if (Math.abs(intake_motor_power_) > 0.01 || Math.abs(shooter_motor_power_) > 0.01) {
                            setMotorsPower(0.0, 0.0) ;
                        }
                    }
                    break;
                case WAIT_SHOOTER1:
                    if (sensor_states_[SENSOR_IDX_SHOOTER]==true)
                        state_[i] = State.WAIT_SHOOTER0;
                    break;
                case WAIT_SHOOTER0:
                    if (sensor_states_[SENSOR_IDX_SHOOTER]==false)
                    {
                        setMotorsPower(0.0, 0.0) ;
                        ball_count_staged_--;
                        state_[i] = State.WAIT_INTAKE;
                        removeBall(i);
                    }
                    break;
                }

                if (prevst != state_[i]) {
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
                if (Math.abs(intake_motor_power_) > 0.01 || Math.abs(shooter_motor_power_) > 0.01) {
                    setMotorsPower(POWER_OFF_, POWER_OFF_);
                    logger.startMessage(MessageType.Debug, getLoggerID()) ;
                    logger.add("ConveyorSubsystem: shutting down as requested.") ;
                    logger.endMessage();
                }

                stop_collect_requested_ = false ;
            }
        }

        putDashboard("ballcount", DisplayType.Always, ball_count_);
    } 

    @Override
    public void run() throws Exception {
        super.run() ;
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

    public void setBypass(boolean bypass) {
        bypass_ = bypass ;
    }

    public void closeExit() {
        exit_.set(ExitCloseState);
    }

    public void openExit() {
        exit_.set(ExitOpenState) ;
    }

    private void setShooterMotor(double power) throws BadMotorRequestException, MotorRequestFailedException {
        shooter_motor_power_ = power ;
        shooter_motor_.set(power) ;
    }

    private void setIntakeMotor(double power) throws BadMotorRequestException, MotorRequestFailedException {
        intake_motor_power_ = power ;
        intake_motor_.set(power) ;
    }

    public void setMotorsPower(double intake, double shooter) throws BadMotorRequestException, MotorRequestFailedException {
        setIntakeMotor(intake);
        setShooterMotor(shooter);
    }

    protected void setStopCollect()
    {
        MessageLogger logger = getRobot().getMessageLogger();
        logger.startMessage(MessageType.Debug, getLoggerID());
        logger.add("Conveyor: stop collect requested.") ;
        logger.endMessage();

        stop_collect_requested_ = true;
    }

    public void setPreloadedBall()
    {
        ball_types_ [0] =  CargoType.Same;
        state_ [0] = State.START_UPTAKE;
        ball_count_ = 1;
    }
        
    protected void setShootMode() throws BadMotorRequestException, MotorRequestFailedException {
        stop_collect_requested_ = false;
        setMotorsPower(intake_motor_on_, shooter_motor_on_) ;
        state_[0] = State.WAIT_SHOOTER1;
    }  

    protected void setCollectMode() throws BadMotorRequestException, MotorRequestFailedException  {
        stop_collect_requested_ = false;
        setMotorsPower(intake_motor_on_, 0.0) ;
    }

    private void removeBall(int ball_idx) 
    {
        ball_count_ --;
        if (0 == ball_idx)
        {
            ball_types_[0] = ball_types_[1];
            state_[0] = state_[1];
        }
        ball_types_[1] =  CargoType.None;
        state_[1] = State.WAIT_INTAKE;
    }
 } ;
