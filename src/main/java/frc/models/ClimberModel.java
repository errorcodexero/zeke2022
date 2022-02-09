package frc.models;

import java.util.Random;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.simulator.models.SimMotorController;
import org.xero1425.simulator.models.SolenoidModel;
import org.xero1425.simulator.models.TankDriveModel;

import edu.wpi.first.hal.simulation.DIODataJNI;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ClimberModel extends SimulationModel {
    private enum State {
        // The climber model is doing nothing
        Idle,

        // The climber model is started
        Started,

        // The climb sequence is started.  In the model we wait for a fixed amount of time
        // given by the model parameter 'first_mid_sensor_delay' and then we set the value
        // of the first sensor to indicate the first side of the climber has hit the mid bar
        SetFirstMidSensor,

        // The first sensor has hit the mid bar.  We wait for a fixed amount of time given by the
        // model parameter 'second_mid_sensor_delay', validating that the correct drive base motor
        // is running, and then set the second sensor indicating that the climber is flush with the 
        // mid bar.
        SetSecondMidSensor,

        // Both sensor have hit the bar.  We wait for the robot to close the grabbers on the A end of
        // the climber to grab around the mid bar.
        WaitForMidGrabbersClosed,

        // The A grabber has grabbed around the mid bar. The action should have started the windmill
        // motor so we watch to see tha the midmill motor is running.
        WindmillSetHighSensors,

        WaitForHighGrabberClosed,

        WaitForMidGrabberOpen,

        WindmillSetTraverseSensors,

        WaitForTraverseGrabberClosed,

        WaitForHighGrabberOpen,

        Complete,
    };

    static private final String LoggerName = "climber-model" ;
    static private final DoubleSolenoid.Value GrabberClosedValue = DoubleSolenoid.Value.kForward ;
    static private final DoubleSolenoid.Value GrabberOpenValue = DoubleSolenoid.Value.kReverse ;

    static private final String DBModelPropName = "dbmodel" ;
    static private final String DBInstPropName = "dbinst" ;
    static private final String FirstSensorDelayPropName = "first_mid_sensor_delay" ;
    static private final String SecondSensorDelayPropName = "second_mid_sensor_delay" ;
    static private final String SecondSensorDBCheckPropName = "second_mid_sensor_db_check" ;
    static private final String WindMillCheckTime = "windmill_check" ; 
    static private final String WindMillDoneTime = "windmill_done" ;
    static private final String MidGrabberCloseTime = "mid_grabber_close_time" ;
    static private final String HighGrabberCloseTime = "high_grabber_close_time" ;
    static private final String MidGrabberOpenTime = "mid_grabber_close_time" ;
    static private final String TraverseGrabberCloseTime = "high_grabber_close_time" ;
    static private final String HighGrabberOpenTime = "mid_grabber_close_time" ;

    static private final String TouchLeftMidIO = "touch_left_mid_io" ;
    static private final String TouchLeftHighIO = "touch_left_high_io" ;
    static private final String TouchLeftTraverseIO = "touch_left_traverse_io" ;
    static private final String TouchRightMidIO = "touch_right_mid_io" ;
    static private final String TouchRightHighIO = "touch_right_high_io" ;
    static private final String TouchRightTraverseIO = "touch_right_traverse_io" ;

    static private final String LeftAModulePropName = "left_a_module" ;
    static private final String LeftAForwardPropName = "left_a_forward" ;
    static private final String LeftAReversePropName = "left_a_reverse" ;

    static private final String RightAModulePropName = "right_a_module" ;
    static private final String RightAForwardPropName = "right_a_forward" ;
    static private final String RightAReversePropName = "right_a_reverse" ;

    static private final String LeftBModulePropName = "left_b_module" ;
    static private final String LeftBForwardPropName = "left_b_forward" ;        
    static private final String LeftBReversePropName = "left_b_forward" ;       

    static private final String RightBModulePropName = "right_b_module" ;
    static private final String RightBForwardPropName = "right_b_forward" ;        
    static private final String RightBReversePropName = "right_b_reverse" ;    

    private Random random_;
    private State state_;
    private SimMotorController motor_;

    private SolenoidModel solenoid_model_ ;

    private int grabber_left_a_;
    private int grabber_left_b_;
    private int grabber_right_a_;
    private int grabber_right_b_;

    private double phase_start_time_;
    private int sensor_side_;

    private int touch_left_mid_io_;
    private boolean touch_left_mid_value_ = false;
    private int touch_left_traverse_io_;
    private boolean touch_left_traverse_value_ = false;
    private int touch_right_mid_io_;
    private boolean touch_right_mid_value_ = false;
    private int touch_right_traverse_io_;
    private boolean touch_right_traverse_value_ = false;
    private int touch_left_high_io_;
    private boolean touch_left_high_value_ = false;
    private int touch_right_high_io_;
    private boolean touch_right_high_value_ = false;

    private TankDriveModel dbmodel_ ;

    private double first_sensor_time_ ;
    private double second_sensor_time_ ;
    private double second_sencor_db_check_time_ ;
    private double windmill_check_time_ ;
    private double windmill_done_time_ ;
    private double wait_for_mid_grabber_close_time_ ;
    private double wait_for_high_grabber_close_time_ ;
    private double wait_for_mid_grabber_open_time_ ;
    private double wait_for_traverse_grabber_close_time_ ;
    private double wait_for_high_grabber_open_time_ ;

    private int logger_id_ ;

    private int msg_count_ ;

    public ClimberModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst);

        random_ = new Random();
        state_ = State.Idle;

        msg_count_ = 0 ;
    }

    @Override
    public boolean create() {

        solenoid_model_ = SolenoidModel.getInstance(getEngine().getMessageLogger()) ;

        logger_id_ = getEngine().getRobot().getMessageLogger().registerSubsystem(LoggerName) ;
               
        motor_ = new SimMotorController(this, "windmill");
        if (!motor_.createMotor())
            return false;

        String dbmodel ;
        String dbinst ;

        try {
            dbmodel = getStringProperty(DBModelPropName) ;
            dbinst = getStringProperty(DBInstPropName) ;

            first_sensor_time_ = getDoubleProperty(FirstSensorDelayPropName) ;
            second_sensor_time_ = getDoubleProperty(SecondSensorDelayPropName) ;
            second_sencor_db_check_time_ = getDoubleProperty(SecondSensorDBCheckPropName) ;
            windmill_check_time_ = getDoubleProperty(WindMillCheckTime) ;
            windmill_done_time_ = getDoubleProperty(WindMillDoneTime) ;
            wait_for_mid_grabber_close_time_ = getDoubleProperty(MidGrabberCloseTime) ;
            wait_for_high_grabber_close_time_ = getDoubleProperty(HighGrabberCloseTime) ;
            wait_for_mid_grabber_open_time_ = getDoubleProperty(MidGrabberOpenTime) ;
            wait_for_traverse_grabber_close_time_ = getDoubleProperty(TraverseGrabberCloseTime) ;
            wait_for_high_grabber_open_time_ = getDoubleProperty(HighGrabberOpenTime) ;

            if (second_sensor_time_ <= second_sencor_db_check_time_) {
                MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" the parameter ").addQuoted(SecondSensorDelayPropName) ;
                logger.add(" must be greater than the parameter ").addQuoted(SecondSensorDBCheckPropName) ;
                logger.endMessage();  
                return false ;
            }

            grabber_left_a_ = createGrabber(LeftAModulePropName, LeftAForwardPropName, LeftAReversePropName) ;
            grabber_left_b_ = createGrabber(LeftBModulePropName, LeftBForwardPropName, LeftBReversePropName) ;
            grabber_right_a_ = createGrabber(RightAModulePropName, RightAForwardPropName, RightAReversePropName) ;
            grabber_right_b_ = createGrabber(RightBModulePropName, RightBForwardPropName, RightBReversePropName) ;

            touch_left_mid_io_ = getIntProperty(TouchLeftMidIO) ;
            touch_left_high_io_ = getIntProperty(TouchLeftHighIO) ;
            touch_left_traverse_io_ = getIntProperty(TouchLeftTraverseIO) ;

            touch_right_mid_io_ = getIntProperty(TouchRightMidIO) ;
            touch_right_high_io_ = getIntProperty(TouchRightHighIO) ;
            touch_right_traverse_io_ = getIntProperty(TouchRightTraverseIO) ;

        }
        catch(Exception ex) {
            return false ;
        }
        
        dbmodel_ = (TankDriveModel)getEngine().findModel(dbmodel, dbinst) ;
        if (dbmodel_ == null) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" the referenced tank drive model ").addQuoted(dbmodel) ;
            logger.add(" instance ").addQuoted(dbinst).add(" does not exist") ;
            logger.endMessage();                    
            return false ;
        }

        setCreated();
        setSensors();
        return true;
    }

    public void run(double dt) {
        switch (state_) {
            case Idle:
                break;

            case Started:
                doStarted() ;
                break;

            case SetFirstMidSensor:
                setFirstMidSensor() ;
                break;

            case SetSecondMidSensor:      
                setSecondMidSensor() ;
                break ;

            case WaitForMidGrabbersClosed:
                waitForMidGrabberClosed() ;
                break ;

            case WindmillSetHighSensors:
                setHighBothSensors() ;
                break ;

            case WaitForHighGrabberClosed:
                waitForHighGrabberClosed() ;
                break ;

            case WaitForMidGrabberOpen:
                waitForMidGrabberOpen();
                break ;
        
            case WindmillSetTraverseSensors:
                setTraverseBothSensors() ;
                break ;
        
            case WaitForTraverseGrabberClosed:
                waitForTraverseGrabberClosed() ;
                break ;
        
            case WaitForHighGrabberOpen:
                waitForHighGrabberOpen() ;
                break ;

            case Complete:
                break ;
        }
    }

    public boolean processEvent(String name, SettingsValue value) {
        if (name.equals("start")) {
            try {
                if (value.isBoolean() && value.getBoolean() == true)
                    state_ = State.Started;
            } catch (BadParameterTypeException e) {
            }
        }
        return true;
    }

    private void doStarted() {
        //
        // The climber model sequencing has been triggered. Select the left side
        // right side, or both sides at random and move to the next state.
        //
        double r = random_.nextDouble() ;
        if (r > 0.9) {
            // Both sensor at once, 10% of the time
            sensor_side_ = 2 ;

        }
        else if (r > 0.45) {
            // Right sensor
            sensor_side_ = 1 ;

        }
        else {
            // Left sensor
            sensor_side_ = 0 ;
        }

        MessageLogger logger = getEngine().getMessageLogger() ;
        logger.startMessage(MessageType.Info, logger_id_) ;
        logger.add("event: model ").addQuoted(getModelName());
        logger.add("instance ").addQuoted(getInstanceName());
        logger.add(":starting the climber model, waiting on delay to first sensor switch") ;
        logger.endMessage();

        phase_start_time_ = getRobotTime();
        state_ = State.SetFirstMidSensor;
    }

    private void setFirstMidSensor() {
        //
        // Wait for a specific amount of time and then trigger the sensor or sensors that detect the 
        // mid bar.
        //
        if (getRobotTime() - phase_start_time_ > first_sensor_time_) {
            String which  = null ;

            if (sensor_side_ == 0) {
                touch_left_mid_value_ = true;
                which = ": the left side" ;
            } else if (sensor_side_ == 1) {
                which = ": the right side" ;
                touch_right_mid_value_ = true;
            } else if (sensor_side_ == 2) {
                which = ": both sides" ;
                touch_right_mid_value_ = true;
                touch_left_mid_value_ = true;
            }

            setSensors();

            if (which != null) {
                MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Info, logger_id_) ;
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(which).add(" of the climber hit the bar (sensor 1)") ;
                logger.endMessage();
            }

            sensor_side_ = 1 - sensor_side_;
            phase_start_time_ = getRobotTime();
            state_ = State.SetSecondMidSensor;
            msg_count_ = 0 ;
        }
    }

    private void setSecondMidSensor() {          
        //
        // In this state, we are waiting to set the second sensor on the mid bar for
        // the original hook up to the bar.  We also check that the side of the drive base associated
        // with the sensor that has not connected the bar is moving.
        //
        if (getRobotTime() - phase_start_time_ > second_sencor_db_check_time_) {
            MessageLogger logger = getEngine().getMessageLogger() ;

            if (msg_count_ < 1) {
                // Check the DB
                if (sensor_side_ == 0) {
                    // We are waiting on the left side of the robot
                    if (dbmodel_.getLeftPower() <= 0.01) {
                        logger.startMessage(MessageType.Error);
                        logger.add("event: model ").addQuoted(getModelName());
                        logger.add(" instance ").addQuoted(getInstanceName());
                        logger.add(" the left side of the drive base is not powered when expected") ;
                        logger.endMessage();
                        getEngine().addAssertError();
                    }
                }
                else {
                    // We are waiting on the right side of the robot
                    if (dbmodel_.getRightPower() <= 0.01) {
                        logger.startMessage(MessageType.Error);
                        logger.add("event: model ").addQuoted(getModelName());
                        logger.add(" instance ").addQuoted(getInstanceName());
                        logger.add(" the right side of the drive base is not powered when expected") ;
                        logger.endMessage();
                        getEngine().addAssertError();
                    }
                }
                msg_count_++ ;
            }
        }

        // Check the time for the second sensor
        if (getRobotTime() - phase_start_time_ > second_sensor_time_) {
            String which = null ;

            if (sensor_side_ == 0) {
                touch_left_mid_value_ = true;
                which = "left" ;
            }
            else {
                touch_right_mid_value_ = true;
                which = "right" ;
            }

            if (which != null) {
                MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Info, logger_id_) ;
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(": the " ) ;
                logger.add(which).add(" side of the climber hit the bar (sensor 2)") ;
                logger.endMessage();
            }

            setSensors();
            state_ = State.WaitForMidGrabbersClosed ;
            phase_start_time_ = getRobotTime() ;
            msg_count_ = 0 ;
        }
    }

    private void waitForMidGrabberClosed() {
        MessageLogger logger = getEngine().getMessageLogger() ;

        //
        // Wait for the robot code to connect the mid level grabber to the mid level bar.
        //
        if (msg_count_ < 1) {
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": waiting for the grabber A to grab mid bar") ;
            logger.endMessage();
            msg_count_++ ;
        }

        if (solenoid_model_.getDoubleSolenoidState(grabber_left_a_) == GrabberClosedValue && 
                solenoid_model_.getDoubleSolenoidState(grabber_right_a_) == GrabberClosedValue) {
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": both left and right A grabbers closed on the bar") ;
            logger.endMessage();

            state_ = State.WindmillSetHighSensors ;
            phase_start_time_ = getRobotTime() ;
            msg_count_= 0 ;
        }
        else if (getRobotTime() - phase_start_time_ > wait_for_mid_grabber_close_time_) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" the mid bar grabbers did not grab the mid bar in the given timeout") ;
            logger.endMessage();
            getEngine().addAssertError();            
        }
    }

    private void setHighBothSensors() {
        if (getRobotTime() - phase_start_time_ > windmill_check_time_) {

            if (motor_.getPower() < 0.01 && msg_count_ < 1) {
                //
                // The windmill is turning
                //
                MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error, logger_id_) ;
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(": windmill motor not running when expected") ;
                logger.endMessage();

                msg_count_++ ;
            }
        }

        if (getRobotTime() - phase_start_time_ > windmill_done_time_) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": windmill phase one complete, setting high sensors") ;
            logger.endMessage();

            touch_left_high_value_ = true ;
            touch_right_high_value_ = true ;
            setSensors();
            msg_count_ = 0 ;

            phase_start_time_ = getRobotTime() ;
            state_ = State.WaitForHighGrabberClosed ;
        }
    }

    private void waitForHighGrabberClosed() {
        MessageLogger logger = getEngine().getMessageLogger() ;

        if (msg_count_ < 1) {
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": waiting for the grabber A to grab mid bar") ;
            logger.endMessage();
            msg_count_++ ;
        }

        if (solenoid_model_.getDoubleSolenoidState(grabber_left_b_) == GrabberClosedValue && 
                solenoid_model_.getDoubleSolenoidState(grabber_right_b_) == GrabberClosedValue) {
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": both left and right A grabbers closed on the bar") ;
            logger.endMessage();

            state_ = State.WaitForMidGrabberOpen ;
            phase_start_time_ = getRobotTime() ;
            msg_count_= 0 ;
        }
        else if (getRobotTime() - phase_start_time_ > wait_for_high_grabber_close_time_) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" the hight bar grabbers did not grab the mid bar in the given timeout") ;
            logger.endMessage();
            getEngine().addAssertError();            
        }
    }

    private void waitForMidGrabberOpen() {
        MessageLogger logger = getEngine().getMessageLogger() ;

        if (msg_count_ < 1) {
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": waiting for the grabber A to grab mid bar") ;
            logger.endMessage();
            msg_count_++ ;
        }

        if (solenoid_model_.getDoubleSolenoidState(grabber_left_a_) == GrabberOpenValue && 
                solenoid_model_.getDoubleSolenoidState(grabber_right_a_) == GrabberOpenValue) {
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": both left and right A grabbers closed on the bar") ;
            logger.endMessage();

            state_ = State.WindmillSetTraverseSensors ;
            phase_start_time_ = getRobotTime() ;
            msg_count_= 0 ;
        }
        else if (getRobotTime() - phase_start_time_ > wait_for_mid_grabber_open_time_) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" the hight bar grabbers did not grab the mid bar in the given timeout") ;
            logger.endMessage();
            getEngine().addAssertError();            
        }
    }

    private void setTraverseBothSensors() {
        if (getRobotTime() - phase_start_time_ > windmill_check_time_) {

            if (motor_.getPower() < 0.01 && msg_count_ < 1) {
                //
                // The windmill is turning
                //
                MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error, logger_id_) ;
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(": windmill motor not running when expected") ;
                logger.endMessage();

                msg_count_++ ;
            }
        }

        if (getRobotTime() - phase_start_time_ > windmill_done_time_) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": windmill phase one complete, setting high sensors") ;
            logger.endMessage();

            touch_left_traverse_value_ = true ;
            touch_right_traverse_value_ = true ;
            setSensors();
            msg_count_ = 0 ;

            phase_start_time_ = getRobotTime() ;
            state_ = State.WaitForHighGrabberClosed ;
        }
    }

    private void waitForTraverseGrabberClosed() {
        MessageLogger logger = getEngine().getMessageLogger() ;

        if (msg_count_ < 1) {

            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": waiting for the grabber A to grab mid bar") ;
            logger.endMessage();
            msg_count_++ ;
        }

        if (solenoid_model_.getDoubleSolenoidState(grabber_left_a_) == GrabberClosedValue && 
                solenoid_model_.getDoubleSolenoidState(grabber_right_a_) == GrabberClosedValue) {
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": both left and right A grabbers closed on the bar") ;
            logger.endMessage();

            state_ = State.WaitForHighGrabberOpen ;
            phase_start_time_ = getRobotTime() ;
            msg_count_= 0 ;
        }
        else if (getRobotTime() - phase_start_time_ > wait_for_mid_grabber_open_time_) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" the hight bar grabbers did not grab the mid bar in the given timeout") ;
            logger.endMessage();
            getEngine().addAssertError();            
        }
    }

    private void waitForHighGrabberOpen() {
        if (msg_count_ < 1) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": waiting for the grabber A to grab mid bar") ;
            logger.endMessage();
            msg_count_++ ;
        }

        if (solenoid_model_.getDoubleSolenoidState(grabber_left_b_) == GrabberOpenValue && 
                solenoid_model_.getDoubleSolenoidState(grabber_right_b_) == GrabberOpenValue) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": both left and right A grabbers closed on the bar") ;
            logger.endMessage();

            state_ = State.Complete ;
            msg_count_= 0 ;
        }
    }

    private void setSensors() {
        DIODataJNI.setValue(touch_left_mid_io_, touch_left_mid_value_);
        DIODataJNI.setValue(touch_right_mid_io_, touch_right_mid_value_);

        DIODataJNI.setValue(touch_left_high_io_, touch_left_high_value_);
        DIODataJNI.setValue(touch_right_high_io_, touch_right_high_value_);

        DIODataJNI.setValue(touch_left_traverse_io_, touch_left_traverse_value_);
        DIODataJNI.setValue(touch_right_traverse_io_, touch_right_traverse_value_);
    }

    private int createGrabber(String modname, String forname, String revname) throws Exception {
        int module = 0, forward = 0, reverse = 0 ;

        try {
            module = getIntProperty(modname) ;
            forward = getIntProperty(forname) ;
            reverse = getIntProperty(revname) ;
        }
        catch(Exception ex) {
            return -1 ;
        }

        return solenoid_model_.getDoubleSolenoid(module, forward, reverse) ;
    }
}
