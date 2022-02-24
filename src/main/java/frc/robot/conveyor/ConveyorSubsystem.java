package frc.robot.conveyor;

import java.util.ArrayList;
import java.util.List;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.pneumatics.XeroSolenoid;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.SettingsValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.zeke_color_sensor.ZekeColorSensor;
import frc.robot.zeke_color_sensor.ZekeColorSensor.CargoType;

public class ConveyorSubsystem extends Subsystem {

    // States in the state machine, basically the last sensor seen
    private enum State {
        INTAKE,
        COLORSENSOR,
        EXIT,
        CHIMNEY,
        REMOVE,
        WANTPARK,
        PARKED
    }

    private class BallInfo
    {
        public CargoType type_ ;
        public State state_ ;
        public int which_ ;

        public BallInfo() {
            type_ = CargoType.None ;
            state_ = State.INTAKE ;
            which_ = which_ball_++ ;
        }

        public String toString() {
            return Integer.toString(which_) + " " + type_.toString() + " " + state_.toString() ;
        }
    }

    private enum Mode {
        IDLE,
        COLLECT,
        SHOOT
    }

    // The total number of sensors
    private static final int SENSOR_COUNT = 4;

    // The indexes of the various sensors
    private static final int SENSOR_IDX_INTAKE = 0;
    private static final int SENSOR_IDX_EXIT = 1;
    private static final int SENSOR_IDX_CHIMNEY = 2;
    private static final int SENSOR_IDX_SHOOTER = 3;

    // The maximum ball count
    private static final int MAX_BALL_COUNT = 2 ;

    // The state of the exit XeroSolenoid when the exit portal is closed
    private static final boolean ExitCloseState = false ;

    // The state of the exit XeroSolenoid when the exit portal is open
    private static final boolean ExitOpenState = true ;

    // The names of the sensors
    private static final String[] SensorNames = { "intake", "exit", "chimney", "shooter"} ;

    // The name of the subsystem
    public static final String SubsystemName = "conveyor";

    // If true, print detailed information every robot loop
    private static final boolean PrintDetailed = false ;

    // If true, print sensor info and ball state info on change
    private static final boolean PrintOnChanged = true ;

    private static int which_ball_ = 0 ;

    private boolean bypass_ ;                       // Bypass the conveyor state machine
    private Mode mode_ ;                            // The mode we are in
    private boolean stop_requested_;                // If true, stop the current conveyor operation as soon as possible
    private List<BallInfo> balls_info_ ;            // The state of the balls
    private BallInfo parked_ ;                      // The ball, if any, parked in the chimney

    private MotorController intake_motor_ ;         // The motor for the flash intake piece
    private double intake_motor_power_ ;            // The currently assigned motor power
    private double intake_motor_on_ ;               // The power when the motor is on

    private MotorController shooter_motor_ ;        // The motor for the shooter side of the conveyor
    private double shooter_motor_power_ ;           // The currently assigned motor power
    private double shooter_motor_on_ ;              // The power when the motor is on

    private XeroSolenoid exit_ ;                    // The solenoid that controls the exit door
    private boolean exit_door_state_ ;              // TRUE means door is open

    private ZekeColorSensor color_sensor_;          // The color sensor for the conveyor

    private String prev_sensors_state_ ;            // The previous state string for the conveyor
    private String prev_balls_state_ ;              // The previous balls state for the conveyor
 
    private CargoType cargo_type_ ;                 // The current robot loop cargo type
    private CargoType prev_cargo_type_ ;            // The previous robot loop cargo type

    private DigitalInput[] sensors_;                // The array of ball detect sensors
    private boolean[] sensor_states_;               // The states of ball detect sensors
    private boolean[] sensor_states_prev_;          // The states of ball detect sensors
    

    public ConveyorSubsystem(Subsystem parent, ZekeColorSensor color) throws Exception {
        super(parent, SubsystemName);
      
        color_sensor_ = color;

        bypass_ = false ;
        prev_sensors_state_ = "" ;

        sensors_ = new DigitalInput[SENSOR_COUNT];
        sensor_states_ = new boolean[SENSOR_COUNT];
        sensor_states_prev_ = new boolean[SENSOR_COUNT];

        exit_door_state_ = false ;
        stop_requested_ = false ;

        balls_info_ = new ArrayList<BallInfo>() ;
        parked_ = null ;

        mode_ = Mode.IDLE ;

        attachHardware() ;
    }

    @Override
    public SettingsValue getProperty(String name) {
        SettingsValue v = null ;

        if (name.equals("ball-count")) {
            v = new SettingsValue(getBallCount()) ;
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
        MessageLogger logger = getRobot().getMessageLogger() ;

        if (PrintOnChanged) {
            String bstate = "" ;

            for(BallInfo b : balls_info_) {
                bstate += "[" + b.toString() + "]" ;
            }

            if (!bstate.equals(prev_balls_state_)) {
                logger.startMessage(MessageType.Debug, getLoggerID()) ;
                logger.add("Balls:").add(bstate) ;
                logger.endMessage();
                prev_balls_state_ = bstate ;
            }
        }

        readSensors();

        if (!bypass_) {

            if (stop_requested_ && isValidStopState()) {
                mode_ = Mode.IDLE ;
                stop_requested_ = false ;
                setIntakeMotor(0.0);
                setShooterMotor(0.0);
                closeExit();
                return ;
            }

            if (risingEdge(SENSOR_IDX_INTAKE)) {
                //
                // There is a rising edge on the intake sensor, add a new ball
                // to the conveyor
                //
                balls_info_.add(new BallInfo()) ;
            }

            if (fallingEdge(SENSOR_IDX_SHOOTER)) {
                if (parked_ != null) {
                    //
                    // The parked ball has left the robot
                    //
                    parked_ = null ;
                }
            }

            for(BallInfo binfo : balls_info_) {
                switch(binfo.state_) {
                    case INTAKE:
                        if (changeToCargoType()) {
                            binfo.type_ = cargo_type_ ;
                            binfo.state_ = State.COLORSENSOR ;
                        }
                        break ;

                    case COLORSENSOR:
                        if (risingEdge(SENSOR_IDX_EXIT)) {
                            binfo.state_ = State.EXIT ;
                        }
                        else if (risingEdge(SENSOR_IDX_CHIMNEY)) {
                            binfo.state_ = State.CHIMNEY ;
                        }
                        break ;

                    case EXIT:
                        if (fallingEdge(SENSOR_IDX_EXIT)) {
                            binfo.state_ = State.REMOVE ;
                        }
                        break ;

                    case CHIMNEY:
                        if (fallingEdge(SENSOR_IDX_CHIMNEY)) {
                            binfo.state_ = State.WANTPARK ;
                        }
                        break ;

                    case WANTPARK:
                        break ;

                    case PARKED:
                        break ;

                    case REMOVE:
                        assert false ;
                }
            }

            if (!balls_info_.isEmpty()) {
                if (balls_info_.get(0).state_ == State.REMOVE) {
                    removeBall();
                }
                else if (balls_info_.get(0).state_ == State.WANTPARK && parked_ == null) {
                    parked_ = balls_info_.get(0) ;
                    parked_.state_ = State.PARKED ;
                    balls_info_.remove(0) ;
                }
            }

            setDoorState() ;
            setMotorState() ;
        }

        putDashboard("ballcount", DisplayType.Always, getBallCount());
    }

    public int getBallCount() {
        int ret = 0 ;

        for(BallInfo b : balls_info_) {
            if (b.type_ == CargoType.Same)
                ret++ ;
        }

        if (parked_ != null)
            ret++ ;

        return ret ;
    }

    public void resetBallCount() {
        parked_ = null ;
        balls_info_.clear();
    }

    @Override
    public void run() throws Exception {
        super.run() ;
    }
    
    public boolean isFull() {
        return getBallCount() == MAX_BALL_COUNT ;
    }

    public boolean isEmpty() {
        return getBallCount() == 0 ;
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
        if (PrintDetailed) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getLoggerID()).add("setShooterMotor").add("power", power).endMessage();
        }
        shooter_motor_power_ = power ;
        shooter_motor_.set(power) ;
    }

    private void setIntakeMotor(double power) throws BadMotorRequestException, MotorRequestFailedException {
        if (PrintDetailed) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getLoggerID()).add("setIntakeMotor").add("power", power).endMessage();
        }
        intake_motor_power_ = power ;
        intake_motor_.set(power) ;
    }

    public void setMotorsPower(double intake, double shooter) throws BadMotorRequestException, MotorRequestFailedException {
        setIntakeMotor(intake);
        setShooterMotor(shooter);
    }

    protected void setStopRequest()
    {
        MessageLogger logger = getRobot().getMessageLogger();
        logger.startMessage(MessageType.Debug, getLoggerID());
        logger.add("Conveyor: stop requested.") ;
        logger.endMessage();

        stop_requested_ = true;
    }

    public void setPreloadedBall()
    {
        BallInfo b = new BallInfo() ;
        b.state_ = State.COLORSENSOR ;
        b.type_ = CargoType.Same ;
        balls_info_.add(b) ;
    }

    public boolean isIdle() {
        return mode_ == Mode.IDLE ;
    }
        
    protected void setShootMode() throws BadMotorRequestException, MotorRequestFailedException {
        mode_ = Mode.SHOOT ;
        stop_requested_ = false;
        setMotorsPower(intake_motor_on_, shooter_motor_on_) ;
    }  

    protected void setCollectMode() throws BadMotorRequestException, MotorRequestFailedException  {
        mode_ = Mode.COLLECT ;
        stop_requested_ = false;
        setMotorsPower(intake_motor_on_, 0.0) ;
    }

    private void removeBall() 
    {
        if(PrintOnChanged) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getLoggerID()) ;
            logger.add("removing ball").endMessage();
        }

        balls_info_.remove(0) ;
    }

    private boolean isValidStopState() {
        if (balls_info_.size() == 0)
            return true ;

        if (PrintOnChanged) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getLoggerID()) ;
            logger.add("ValidStopState:");
            logger.add("parked", (parked_ != null ? "true" : "false")) ;
            logger.endMessage();
        }

        //
        // The only valid stop state is if the horizontal conveyor is empty, or if I already have a ball of my
        // color parked in the chimney, and the current ball in the conveyor is my color, and it has tripped the
        // chimney sensor (so we park it here).
        //
        if (parked_ != null && balls_info_.get(0).type_ == CargoType.Same && balls_info_.get(0).state_ == State.CHIMNEY)
            return true ;

        return false ;
    }

    private void setMotorState() throws BadMotorRequestException, MotorRequestFailedException {
        if (mode_ == Mode.SHOOT) {
            setShooterMotor(shooter_motor_on_);
        }
        else {
            if (parked_ != null) {
                //
                // We have a ball in the chimney, stop the shooter motor
                //
                setShooterMotor(0.0);
            }
            else if (balls_info_.size() == 0) {
                //
                // There are no balls in the conveyor to be moved, stop the shooter
                //            
                setShooterMotor(0.0);
            }
            else if (balls_info_.get(0).type_ == CargoType.Opposite || balls_info_.get(0).type_ == CargoType.None) {
                //
                // The ball at the front of the conveyor is going to the exit, stop the shooter
                //
                setShooterMotor(0.0);
            }
            else {
                //
                // We have a ball, it is our color, and we don't already have one parked,
                // turn on the shooter motor
                //
                setShooterMotor(shooter_motor_on_);
            }
        }

        //
        // The collect action starts the intake motor.  We never turn it on in the conveyor state machine.  We just turn it off
        // when it is time.
        //

        if (parked_ != null && balls_info_.size() > 0 && balls_info_.get(0).type_ == CargoType.Same && balls_info_.get(0).state_ == State.CHIMNEY) {
            //
            // We have a ball in the chimney laready, and we have a ball in the horizontal conveyor, and the
            // ball in the horizontal conveyor is the same color as the robot, and it has triggered the chimney sensor, 
            // shut down the motors
            //
            setIntakeMotor(0.0);
        }
    }

    private void setDoorState() {

        boolean state = ExitCloseState ;

        if (balls_info_.size() > 0) {
            BallInfo b = balls_info_.get(0) ;
            if (b.type_ == CargoType.Opposite)
                state = ExitOpenState ;
            else if (b.type_ == CargoType.Same)
                state = ExitCloseState ;
            else
                state = exit_door_state_ ;
        }

        if (state != exit_door_state_) {
            if (state == ExitCloseState)
                closeExit();
            else
                openExit();
        }

    }

    private boolean risingEdge(int which) {
        boolean ret = sensor_states_[which] == true && sensor_states_prev_[which] == false ;
        sensor_states_prev_[which] = sensor_states_[which] ;
        return ret ;
    }

    private boolean fallingEdge(int which) {
        boolean ret = sensor_states_[which] == false && sensor_states_prev_[which] == true ;
        sensor_states_prev_[which] = sensor_states_[which] ;
        return ret ;        
    }

    private boolean changeToCargoType() {
        boolean ret = cargo_type_ != CargoType.None && prev_cargo_type_ == CargoType.None ;
        prev_cargo_type_ = cargo_type_ ;
        return ret ;
    }

    //
    // Read the beam break and color sensors for this robot loop
    //
    private void readSensors() {
        MessageLogger logger = getRobot().getMessageLogger() ;
        String senstr = "" ;

        for (int i = 0; i < SENSOR_COUNT; i++) {
            //
            // Get the sensor state
            //
            sensor_states_prev_[i] = sensor_states_[i];
            sensor_states_[i] = !sensors_[i].get();

            if (PrintOnChanged) {
                senstr += " " ;
                senstr += SensorNames[i] ;
                senstr += " " ;
                senstr += (sensor_states_[i] ? "true" : "false") ;
            }

            String sname = SensorNames[i] ;
            putDashboard(sname, Subsystem.DisplayType.Verbose, sensor_states_[i]);
        }

        prev_cargo_type_ = cargo_type_ ;
        cargo_type_ =  color_sensor_.getCargoType(color_sensor_.getConveyorIndex());

        if (PrintOnChanged) {
            senstr += ", color " + cargo_type_.toString() ;
            if (!senstr.equals(prev_sensors_state_)) {
                logger.startMessage(MessageType.Debug, getLoggerID()).add(senstr).endMessage(); 
            }
        }

        prev_sensors_state_ = senstr ;
    }    

    //
    // Attach all of the required hardware to the conveyor
    //
    private void attachHardware() throws BadParameterTypeException, MissingParameterException {
        exit_ = new XeroSolenoid(this, "exit") ;

        intake_motor_ = getRobot().getMotorFactory().createMotor("intake", "subsystems:conveyor:hw:motors:intake");
        intake_motor_power_ = 0.0 ;
        intake_motor_on_ = getSettingsValue("power:intake").getDouble() ;

        shooter_motor_ = getRobot().getMotorFactory().createMotor("shooter", "subsystems:conveyor:hw:motors:shooter");
        shooter_motor_power_ = 0.0 ;
        shooter_motor_on_ = getSettingsValue("power:shooter").getDouble() ;

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
 } ;

