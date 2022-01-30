package frc.models;

import java.util.Random;

import org.xero1425.base.pneumatics.XeroDoubleSolenoid;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.simulator.models.SimMotorController;
import org.xero1425.simulator.models.TankDriveModel;

import edu.wpi.first.hal.simulation.DIODataJNI;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;

public class ClimberModel extends SimulationModel {
    private enum State {
        // The climber model is doing nothing
        Idle,

        // The climber model is started
        Started,

        // The climb sequence is started, we are looking for the first
        // one of the two touch sensors
        WaitingMidFirstSensor,

        // The first sensor has hit the bar, now we are waiting a fixed amount
        // of time for the second sensor to hit.
        //
        // Checks: Be sure the correct side of the drive base is running based on
        //         which sensor hit the bar
        WaitingMidSecondSensor,

        // Both sensor have hit the bar.  We wait for the grabbers to close around
        // the bar.
        WaitForAGrabbersClosed,
    };

    static private final String DBModelPropName = "dbmodel" ;
    static private final String DBInstPropName = "dbinst" ;
    static private final String FirstSensorDelayPropName = "first_sensor_delay" ;
    static private final String SecondSensorDelayPropName = "second_sensor_delay" ;
    static private final String SecondSensorDBCheckPropName = "second_sensor_db_check" ;

    static private final String TouchLeftMidIO = "touch_left_mid_io" ;
    static private final String TouchLeftHighIO = "touch_left_high_io" ;
    static private final String TouchLeftTraverseIO = "touch_left_traverse_io" ;
    static private final String TouchRightMidIO = "touch_right_mid_io" ;
    static private final String TouchRightHighIO = "touch_right_high_io" ;
    static private final String TouchRightTraverseIO = "touch_right_traverse_io" ;

    static private final String LeftAModulePropName = "left_a_module" ;
    static private final String LeftAChannelPropName = "left_a_channel" ;

    static private final String RightAModulePropName = "right_a_module" ;
    static private final String RightAChannelPropName = "right_a_channel" ;

    static private final String LeftBModulePropName = "left_b_module" ;
    static private final String LeftBChannelPropName = "left_b_channel" ;        

    static private final String RightBModulePropName = "right_b_module" ;
    static private final String RightBChannelPropName = "right_b_channel" ;        

    private Random random_;
    private State state_;
    private SimMotorController motor_;

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

    public ClimberModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst);

        random_ = new Random();
        state_ = State.Idle;
    }

    @Override
    public boolean create() {
       
        motor_ = new SimMotorController(this, "motor");
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

            grabber_left_a_ = createGrabber(LeftAModulePropName, LeftAChannelPropName) ;
            grabber_left_b_ = createGrabber(RightAModulePropName, RightAChannelPropName) ;
            grabber_right_a_ = createGrabber(LeftBModulePropName, LeftBChannelPropName) ;
            grabber_right_b_ = createGrabber(RightBModulePropName, RightBChannelPropName) ;

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

            case WaitingMidFirstSensor:
                doWaitingMidFirstSensor() ;
                break;

            case WaitingMidSecondSensor:      
                doWaitingMidSecondSensor() ;        
                break ;

            case WaitForAGrabbersClosed:
                doWaitForAGrabberClosed() ;
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
        // The climber model sequencing has been triggered.  We wait for an amount
        // of time given by the FirstSensorDelayPropName property.
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

        phase_start_time_ = getRobotTime();
        state_ = State.WaitingMidFirstSensor;
    }

    private void doWaitingMidFirstSensor() {
        //
        // The wait for the first sensor has expired.  Set the first sensor to true and then
        // wait for the time given by the SecondSensorDelayPropName property.
        //
        if (getRobotTime() - phase_start_time_ > first_sensor_time_) {
            if (sensor_side_ == 0) {
                touch_left_mid_value_ = true;
            } else if (sensor_side_ == 1) {
                touch_right_mid_value_ = true;
            } else if (sensor_side_ == 2) {
                touch_right_mid_value_ = true;
                touch_left_mid_value_ = true;
            }
            setSensors();

            sensor_side_ = 1 - sensor_side_;
            phase_start_time_ = getRobotTime();
            state_ = State.WaitingMidSecondSensor;
        }
    }

    private void doWaitingMidSecondSensor() {          
        //
        // In this state, we are waiting to set the second sensor on the mid bar for
        // the original hook up to the bar.  We also check that the side of the drive base assocaited
        // with the sensor that has not connected the bar is moving, but only after a delay given by
        // the property 'SecondSensorDBCheckPropName'.
        //
        if (getRobotTime() - phase_start_time_ > second_sencor_db_check_time_) {
            MessageLogger logger = getEngine().getMessageLogger() ;

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
                    getEngine().exitSimulator();
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
                    getEngine().exitSimulator();
                }
            }
        }

        // Check the time for the second sensor
        if (getRobotTime() - phase_start_time_ > second_sensor_time_) {
            if (sensor_side_ == 0)
                touch_left_mid_value_ = true;
            else
                touch_right_mid_value_ = true;

            setSensors();
            state_ = State.WaitingMidFirstSensor;
        }
    }

    private void doWaitForAGrabberClosed() {
        
    }

    private void setSensors() {
        DIODataJNI.setValue(touch_left_mid_io_, touch_left_mid_value_);
        DIODataJNI.setValue(touch_left_traverse_io_, touch_left_traverse_value_);
        DIODataJNI.setValue(touch_right_mid_io_, touch_right_mid_value_);
        DIODataJNI.setValue(touch_right_traverse_io_, touch_right_traverse_value_);
        DIODataJNI.setValue(touch_left_high_io_, touch_left_high_value_);
        DIODataJNI.setValue(touch_right_high_io_, touch_right_high_value_);
    }

    private int createGrabber(String modname, String channame) throws Exception {
        int module, channel, index ;

        try {
            module = getIntProperty(modname) ;
            channel = getIntProperty(channame) ;
            index = XeroDoubleSolenoid.calcIndex(module, channel) ;
        }
        catch(Exception ex) {
            return -1 ;
        }

        return SimDeviceDataJNI.getSimDeviceHandle(XeroDoubleSolenoid.SimDeviceName + "[" + index + "]") ;
    }

    private int getIntProperty(String name) throws Exception {
        MessageLogger logger = getEngine().getMessageLogger() ;

        if (!hasProperty(name)) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" is missing required property").addQuoted(name);
            logger.endMessage();
            throw new Exception("getIntProperty failed") ;
        }

        SettingsValue value = getProperty(name) ;
        if (!value.isInteger()) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" property ").addQuoted(name).add(" is not an integer");
            logger.endMessage();   
            throw new Exception("getIntProperty failed") ;         
        }

        return value.getInteger() ;
    }

    private double getDoubleProperty(String name) throws Exception {
        MessageLogger logger = getEngine().getMessageLogger() ;

        if (!hasProperty(name)) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" is missing required property").addQuoted(name);
            logger.endMessage();
            throw new Exception("getDoubleProperty failed") ;
        }

        SettingsValue value = getProperty(name) ;
        if (!value.isDouble()) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" property ").addQuoted(name).add(" is not an integer");
            logger.endMessage();   
            throw new Exception("getDoubleProperty failed") ;         
        }

        return value.getDouble() ;
    }

    private String getStringProperty(String name) throws Exception {
        MessageLogger logger = getEngine().getMessageLogger() ;

        if (!hasProperty(name)) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" is missing required property").addQuoted(name);
            logger.endMessage();
            throw new Exception("getStringProperty failed") ;
        }

        SettingsValue value = getProperty(name) ;
        if (!value.isString()) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" property ").addQuoted(name).add(" is not a string");
            logger.endMessage();   
            throw new Exception("getStringProperty failed") ;         
        }

        return value.getString() ;
    }       
}
