package frc.models;

import java.util.ArrayList;
import java.util.List;
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

    static private final String TouchLeftA = "touch_left_a" ;
    static private final String TouchLeftB = "touch_left_b" ;
    static private final String TouchRightA = "touch_right_a" ;
    static private final String TouchRightB = "touch_right_b" ;

    static private final String AModulePropName = "a_module" ;
    static private final String AForwardPropName = "a_forward" ;
    static private final String AReversePropName = "a_reverse" ;

    static private final String BModulePropName = "b_module" ;
    static private final String BForwardPropName = "b_forward" ;
    static private final String BReversePropName = "b_reverse" ;

    private Random random_;
    private State state_;
    private SimMotorController motor_;

    private SolenoidModel solenoid_model_ ;

    private List<Integer> messages_ ;

    private int grabber_a_;
    private int grabber_b_;

    private double phase_start_time_;
    private int sensor_side_;

    private int touch_left_a_;
    private boolean touch_left_a_value_ = false;
    private int touch_right_a_;
    private boolean touch_right_a_value_ = false;
    private int touch_left_b_;
    private boolean touch_left_b_value_ = false;
    private int touch_right_b_;
    private boolean touch_right_b_value_ = false;


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

    private double angle_ ;
    private double degrees_per_second_per_volt_ ;

    private int logger_id_ ;

    public ClimberModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst);

        random_ = new Random();
        state_ = State.Idle;

        messages_ = new ArrayList<Integer>() ;
    }

    private boolean createMotor() throws Exception {
                       
        motor_ = new SimMotorController(this, "windmill");
        if (!motor_.createMotor())
            return false;

        degrees_per_second_per_volt_ = getDoubleProperty("degrees_per_second_per_volt") ;

        return true ;
    }

    @Override
    public boolean create() {

        try {
            if (!createMotor())
                return false ;
        }
        catch(Exception ex) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" - creation failed - ").add(ex.getMessage()) ;
            logger.endMessage();  
        }

        solenoid_model_ = SolenoidModel.getInstance(getEngine().getMessageLogger()) ;
        logger_id_ = getEngine().getRobot().getMessageLogger().registerSubsystem(LoggerName) ;

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

            grabber_a_ = createGrabber(AModulePropName, AForwardPropName, AReversePropName) ;
            grabber_b_ = createGrabber(BModulePropName, BForwardPropName, BReversePropName) ;

            touch_left_a_ = getIntProperty(TouchLeftA) ;
            touch_left_b_ = getIntProperty(TouchLeftB) ;

            touch_right_a_ = getIntProperty(TouchRightA) ;
            touch_right_b_ = getIntProperty(TouchRightB) ;

        }
        catch(Exception ex) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" - creation failed - ").add(ex.getMessage()) ;
            logger.endMessage();    
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

        double power = motor_.getPower() ;
        angle_ += degrees_per_second_per_volt_ * dt * power ;
        double v = angle_ / 360.0 * 7150.488871 ;
        motor_.setEncoder(v);

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
        logger.add(" instance ").addQuoted(getInstanceName());
        logger.add(" :starting the climber model, waiting on delay to first sensor switch") ;
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
                touch_left_a_value_ = true;
                which = ": the left side" ;
            } else if (sensor_side_ == 1) {
                which = ": the right side" ;
                touch_right_a_value_ = true;
            } else if (sensor_side_ == 2) {
                which = ": both sides" ;
                touch_left_a_value_ = true;
                touch_right_a_value_ = true;
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

            // Check the DB
            if (sensor_side_ == 0) {
                // We are waiting on the left side of the robot
                if (dbmodel_.getLeftPower() <= 0.01) {
                    if (!messages_.contains(1)) {
                        logger.startMessage(MessageType.Error);
                        logger.add("event: model ").addQuoted(getModelName());
                        logger.add(" instance ").addQuoted(getInstanceName());
                        logger.add(" the left side of the drive base is not powered when expected") ;
                        logger.endMessage();
                        messages_.add(1) ;
                        getEngine().addAssertError();
                    }
                }
            }
            else {
                // We are waiting on the right side of the robot
                if (dbmodel_.getRightPower() <= 0.01) {
                    if (!messages_.contains(2)) {
                        logger.startMessage(MessageType.Error);
                        logger.add("event: model ").addQuoted(getModelName());
                        logger.add(" instance ").addQuoted(getInstanceName());
                        logger.add(" the right side of the drive base is not powered when expected") ;
                        logger.endMessage();
                        messages_.add(2) ;
                        getEngine().addAssertError();
                    }
                    getEngine().addAssertError();
                }
            }
        }

        // Check the time for the second sensor
        if (getRobotTime() - phase_start_time_ > second_sensor_time_) {
            String which = null ;

            if (sensor_side_ == 0) {
                touch_left_a_value_ = true;
                which = "left" ;
            }
            else {
                touch_right_a_value_ = true;
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
        }
    }

    private void waitForMidGrabberClosed() {
        MessageLogger logger = getEngine().getMessageLogger() ;

        //
        // Wait for the robot code to connect the mid level grabber to the mid level bar.
        //
        if (!messages_.contains(3)) {
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": waiting for the grabber A to grab mid bar") ;
            logger.endMessage();
            messages_.add(3) ;
        }

        if (solenoid_model_.getDoubleSolenoidState(grabber_a_) == GrabberClosedValue) {
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": both left and right A grabbers closed on the mid bar") ;
            logger.endMessage();

            state_ = State.WindmillSetHighSensors ;
            phase_start_time_ = getRobotTime() ;
        }
        else if (getRobotTime() - phase_start_time_ > wait_for_mid_grabber_close_time_) {
            if (!messages_.contains(4)) {
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" the A grabbers did not grab the mid bar in the given timeout") ;
                logger.endMessage();
                messages_.add(4) ;
                getEngine().addAssertError();
            }
            getEngine().addAssertError();            
        }
    }

    private void setHighBothSensors() {
        if (getRobotTime() - phase_start_time_ > windmill_check_time_) {

            if (motor_.getPower() < 0.01) {
                //
                // The windmill is turning
                //
                if (!messages_.contains(5)) {
                    MessageLogger logger = getEngine().getMessageLogger() ;
                    logger.startMessage(MessageType.Error, logger_id_) ;
                    logger.add("event: model ").addQuoted(getModelName());
                    logger.add(" instance ").addQuoted(getInstanceName());
                    logger.add(": windmill motor not running when expected") ;
                    logger.endMessage();
                    messages_.add(5) ;
                    getEngine().addAssertError();
                }
            }
        }

        if (getRobotTime() - phase_start_time_ > windmill_done_time_) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": windmill phase one complete, setting high sensors") ;
            logger.endMessage();

            touch_left_b_value_ = true ;
            touch_right_b_value_ = true ;
            setSensors();

            phase_start_time_ = getRobotTime() ;
            state_ = State.WaitForHighGrabberClosed ;
        }
    }

    private void waitForHighGrabberClosed() {
        MessageLogger logger = getEngine().getMessageLogger() ;

        if (!messages_.contains(6)) {
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": waiting for the grabber B to grab high bar") ;
            logger.endMessage();
            messages_.add(6) ;
        }


        if (solenoid_model_.getDoubleSolenoidState(grabber_b_) == GrabberClosedValue) {
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": both left and right B grabbers closed on the high bar") ;
            logger.endMessage();

            state_ = State.WaitForMidGrabberOpen ;
            phase_start_time_ = getRobotTime() ;
        }
        else if (getRobotTime() - phase_start_time_ > wait_for_high_grabber_close_time_) {            
            if (!messages_.contains(7)) {
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" the B grabbers did not grab the high bar in the given timeout") ;
                logger.endMessage();
                messages_.add(7) ;
                getEngine().addAssertError();            
            }
        }
    }

    private void waitForMidGrabberOpen() {
        MessageLogger logger = getEngine().getMessageLogger() ;

        if (!messages_.contains(8)) {
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": waiting for the grabber A to release the mid bar") ;
            logger.endMessage();
            messages_.add(8) ;
        }

        if (solenoid_model_.getDoubleSolenoidState(grabber_a_) == GrabberOpenValue) {
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": both left and right A grabbers released the mid bar") ;
            logger.endMessage();

            state_ = State.WindmillSetTraverseSensors ;
            phase_start_time_ = getRobotTime() ;
        }
        else if (getRobotTime() - phase_start_time_ > wait_for_mid_grabber_open_time_) {
            if (!messages_.contains(9)) {
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" the A grabbers did not release the mid bar in the given timeout") ;
                logger.endMessage();
                messages_.add(9) ;
                getEngine().addAssertError();
            }
            getEngine().addAssertError();            
        }
    }

    private void setTraverseBothSensors() {
        if (getRobotTime() - phase_start_time_ > windmill_check_time_) {

            if (motor_.getPower() < 0.01) {
                //
                // The windmill is not turning
                //
                if (!messages_.contains(10)) {
                    MessageLogger logger = getEngine().getMessageLogger() ;
                    logger.startMessage(MessageType.Error, logger_id_) ;
                    logger.add("event: model ").addQuoted(getModelName());
                    logger.add(" instance ").addQuoted(getInstanceName());
                    logger.add(": windmill motor not running when expected") ;
                    logger.endMessage();
                    messages_.add(10) ;
                    getEngine().addAssertError();
                }
            }
        }

        if (getRobotTime() - phase_start_time_ > windmill_done_time_) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": windmill phase two complete, setting traverse sensors") ;
            logger.endMessage();

            touch_left_a_value_ = true ;
            touch_right_a_value_ = true ;
            setSensors();

            phase_start_time_ = getRobotTime() ;
            state_ = State.WaitForTraverseGrabberClosed ;
        }
    }

    private void waitForTraverseGrabberClosed() {
        MessageLogger logger = getEngine().getMessageLogger() ;

        if (!messages_.contains(11)) {
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": waiting for the grabber A to grab traverse bar") ;
            logger.endMessage();
            messages_.add(11) ;
        }

        if (solenoid_model_.getDoubleSolenoidState(grabber_a_) == GrabberClosedValue) {
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": both left and right A grabbers closed on the traverse bar") ;
            logger.endMessage();

            state_ = State.WaitForHighGrabberOpen ;
            phase_start_time_ = getRobotTime() ;
        }
        else if (getRobotTime() - phase_start_time_ > wait_for_traverse_grabber_close_time_) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" the A grabbers did not grab the traverse bar in the given timeout") ;
            logger.endMessage();
            getEngine().addAssertError();            
        }
    }

    private void waitForHighGrabberOpen() {
        MessageLogger logger = getEngine().getMessageLogger() ;
        if (!messages_.contains(12)) {
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": waiting for the grabber B to relase high bar") ;
            logger.endMessage();
            messages_.add(12) ;
        }

        if (solenoid_model_.getDoubleSolenoidState(grabber_b_) == GrabberOpenValue) {
            logger.startMessage(MessageType.Info, logger_id_) ;
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(": both left and right B grabbers released the high bar") ;
            logger.endMessage();

            state_ = State.Complete ;
        }
        else if (getRobotTime() - phase_start_time_ > wait_for_high_grabber_open_time_) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" the B bar grabbers did not release the high bar in the given timeout") ;
            logger.endMessage();
            getEngine().addAssertError();            
        }
    }

    private void setSensors() {
        DIODataJNI.setValue(touch_left_a_, touch_left_a_value_);
        DIODataJNI.setValue(touch_right_a_, touch_right_a_value_);
        DIODataJNI.setValue(touch_left_b_, touch_left_b_value_);
        DIODataJNI.setValue(touch_right_b_, touch_right_b_value_);
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
