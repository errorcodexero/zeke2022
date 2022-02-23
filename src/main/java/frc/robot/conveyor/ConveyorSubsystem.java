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

    private class BallInfo
    {
        public CargoType type_ ;
        public State state_ ;
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
    private BallInfo [] balls_ ;

    private MotorController intake_motor_ ;         // The motor for the flash intake piece
    private double intake_motor_power_ ;
    private double intake_motor_on_ ;

    private MotorController shooter_motor_ ;        // The motor for the shooter sidef of the conveyor
    private double shooter_motor_power_ ;
    private double shooter_motor_on_ ;

    private XeroSolenoid exit_ ;    
    private boolean exit_door_state_ ;              // TRUE means door is open

    private ZekeColorSensor color_sensor_;

    private String prev_str_st_ ;
    private CargoType cargo_type_ ;
    private CargoType prev_cargo_type_ ;

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
        prev_str_st_ = "" ;

        sensors_ = new DigitalInput[SENSOR_COUNT];
        sensor_states_ = new boolean[SENSOR_COUNT];
        sensor_states_prev_ = new boolean[SENSOR_COUNT];

        ball_count_ = 0 ;
        ball_count_staged_ = 0;

        exit_door_state_ = false ;

        balls_ = new BallInfo [MAX_BALLS];
        for (int i = 0; i < MAX_BALLS; i++) {
            balls_[i] = new BallInfo() ;
            balls_[i].state_ = State.WAIT_INTAKE ;
            balls_[i].type_ = CargoType.None ;
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

    private void readSensors() {
        MessageLogger logger = getRobot().getMessageLogger() ;
        String senstr = "" ;

        for (int i = 0; i < SENSOR_COUNT; i++) {
            //
            // Get the sensor state
            //
            sensor_states_prev_[i] = sensor_states_[i];
            sensor_states_[i] = !sensors_[i].get();

            senstr += " " ;
            senstr += (sensor_states_[i] ? "true" : "false") ;

            String sname = SensorNames[i] ;
            putDashboard(sname, Subsystem.DisplayType.Always, sensor_states_[i]);
        }

        prev_cargo_type_ = cargo_type_ ;
        cargo_type_ =  color_sensor_.getCargoType(color_sensor_.getConveyorIndex());
        senstr += ", color " + cargo_type_.toString() ;

        if (!senstr.equals(prev_str_st_)) {
            logger.startMessage(MessageType.Debug).add("sensors", senstr).endMessage(); 
            prev_str_st_ = senstr ;
        }
    }

    private boolean risingEdge(int which) {
        return sensor_states_[which] == true && sensor_states_prev_[which] == false ;
    }

    private boolean fallingEdge(int which) {
        return sensor_states_[which] == true && sensor_states_prev_[which] == false ;
    }

    private boolean changedToCargoType(CargoType t) {
        return cargo_type_ == t && prev_cargo_type_ == CargoType.None ;
    }

    @Override
    public void computeMyState() throws Exception {
        MessageLogger logger = getRobot().getMessageLogger() ;


        readSensors();

        if (!bypass_) {
            int loops = Math.min(ball_count_ + 1, MAX_BALLS);

            if (loops == 1 && balls_[0].type_ == CargoType.Same && exit_door_state_) {
                //
                // The state of a second ball may be in a variety of places when the first
                // ball exits, and the state of the system when a first balls exits may not be
                // well enough defined to know to close the exit door.  For instance, if a first
                // ball is leaving the exit, but a second ball has not cross the color sensor, we
                // may not know what to do.
                //
                // This code ensures, that if we get into a state where there is only one ball, and
                // it is the same color of the robot and the exit door is still open, we close it.
                //
                closeExit();
            }

            for (int i = 0; i < loops; i++) {
                State prevst = balls_[i].state_ ;

                switch (prevst) {
                case WAIT_INTAKE:
                    if (risingEdge(SENSOR_IDX_INTAKE))
                    {
                        ball_count_ ++;
                        balls_[i].state_ = State.WAIT_COLOR;
                        balls_[i].type_ = CargoType.None ;

                        // Consume the rising edge, so no other ball can us it
                        sensor_states_prev_[SENSOR_IDX_INTAKE] = sensor_states_[SENSOR_IDX_INTAKE] ;
                    }
                    break;

                case  WAIT_COLOR:
                    if (changedToCargoType(CargoType.Opposite))
                    {
                        exit_.set(ExitOpenState);
                        balls_[i].state_ = State.WAIT_EXIT1;
                        balls_[i].type_ = CargoType.Opposite ;

                        // Consume the color change so no other ball can use it
                        prev_cargo_type_ = cargo_type_ ;
                    }
                    else if (changedToCargoType(CargoType.Same))
                    {
                        balls_[i].state_ = State.START_UPTAKE;
                        balls_[i].type_ = CargoType.Same ;

                        // Consume the color change so no other ball can use it
                        prev_cargo_type_ = cargo_type_ ;
                    }
                    break;
                case  WAIT_EXIT1:
                    if (risingEdge(SENSOR_IDX_EXIT))
                    {
                        balls_[i].state_ = State.WAIT_EXIT0;
                        sensor_states_prev_[SENSOR_IDX_EXIT] = sensor_states_[SENSOR_IDX_EXIT] ;
                    }
                    break;
                case  WAIT_EXIT0:
                    if (fallingEdge(SENSOR_IDX_EXIT))
                    {
                        //
                        // We are about to remove the first ball.  If this is the only ball, close the exit door.
                        //
                        if (loops == 1) {
                            exit_.set(ExitCloseState);
                        }

                        balls_[i].state_ = State.WAIT_INTAKE; 
                        sensor_states_prev_[SENSOR_IDX_EXIT] = sensor_states_[SENSOR_IDX_EXIT] ;
                        removeBall(i);
                    }
                    break;
                case  START_UPTAKE:
                    if (i == 0)
                    {
                        setShooterMotor(shooter_motor_on_);
                        balls_[i].state_ = State.WAIT_CHIMNEY1;
                    }
                    else
                    {
                        ball_count_staged_ = 2;
                        setIntakeMotor(POWER_OFF_);
                    }
                    break;
                case  WAIT_CHIMNEY1:
                    if (risingEdge(SENSOR_IDX_CHIMNEY)) {
                        balls_[i].state_ = State.WAIT_CHIMNEY0;
                        sensor_states_prev_[SENSOR_IDX_CHIMNEY] = sensor_states_[SENSOR_IDX_CHIMNEY] ;
                    }
                    break;
                case  WAIT_CHIMNEY0:
                    if (fallingEdge(SENSOR_IDX_CHIMNEY))
                    {
                        ball_count_staged_ = 1;
                        setShooterMotor(POWER_OFF_);
                        balls_[i].state_ = State.WAIT_SHOOTER;
                        sensor_states_prev_[SENSOR_IDX_CHIMNEY] = sensor_states_[SENSOR_IDX_CHIMNEY] ;
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
                    if (risingEdge(SENSOR_IDX_SHOOTER)) {
                        balls_[i].state_ = State.WAIT_SHOOTER0;
                        sensor_states_prev_[SENSOR_IDX_SHOOTER] = sensor_states_[SENSOR_IDX_SHOOTER] ;
                    }
                    break;
                case WAIT_SHOOTER0:
                    if (fallingEdge(SENSOR_IDX_SHOOTER))
                    {
                        setMotorsPower(0.0, 0.0) ;
                        ball_count_staged_--;
                        balls_[i].state_ = State.WAIT_INTAKE;
                        removeBall(i);
                        
                        sensor_states_prev_[SENSOR_IDX_SHOOTER] = sensor_states_[SENSOR_IDX_SHOOTER] ;                        
                    }
                    break;
                }

                if (prevst != balls_[i].state_) {
                    logger.startMessage(MessageType.Debug, getLoggerID()) ;
                    logger.add("ConveyorSubsystem:") ;
                    logger.add("loop", i) ;
                    logger.add(" ").add(prevst.toString()).add(" --> ").add(balls_[i].state_.toString()) ;
                    logger.endMessage();
                }
            }

            if (stop_collect_requested_ && (
                (ball_count_ == 0) ||
                ((ball_count_ == 1) && (balls_[0].state_ == State.WAIT_SHOOTER)) ||
                ((ball_count_ == 2) && (balls_[0].state_ == State.WAIT_SHOOTER) && (balls_[0].state_ == State.START_UPTAKE))))
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

    public void resetBallCount() {
        while (ball_count_ > 0) {
            removeBall(0);
        }
        ball_count_staged_ = 0 ;
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
        exit_door_state_ = false ;
    }

    public void openExit() {
        exit_.set(ExitOpenState) ;
        exit_door_state_ = true ;
    }

    private void setShooterMotor(double power) throws BadMotorRequestException, MotorRequestFailedException {
        MessageLogger logger = getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug).add("setShooterMotor").add("power", power).endMessage();
        shooter_motor_power_ = power ;
        shooter_motor_.set(power) ;
    }

    private void setIntakeMotor(double power) throws BadMotorRequestException, MotorRequestFailedException {
        MessageLogger logger = getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug).add("setIntakeMotor").add("power", power).endMessage();
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
        balls_[0] = new BallInfo() ;
        balls_[0].state_ = State.START_UPTAKE ;
        balls_[0].type_ = CargoType.Same ;
        ball_count_ = 1;
    }
        
    protected void setShootMode() throws BadMotorRequestException, MotorRequestFailedException {
        stop_collect_requested_ = false;
        setMotorsPower(intake_motor_on_, shooter_motor_on_) ;
        balls_[0].state_ = State.WAIT_SHOOTER1;
    }  

    protected void setCollectMode() throws BadMotorRequestException, MotorRequestFailedException  {
        stop_collect_requested_ = false;
        setMotorsPower(intake_motor_on_, 0.0) ;
    }

    private void removeBall(int ball_idx) 
    {
        MessageLogger logger = getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug) ;
        logger.add("removing ball", ball_idx).endMessage();

        ball_count_--;
        if (0 == ball_idx)
        {
            balls_[0] = balls_[1] ;
        }

        balls_[1] = new BallInfo() ;
        balls_[1].state_ = State.WAIT_INTAKE ;
        balls_[1].type_ = CargoType.None ;
    }
 } ;
