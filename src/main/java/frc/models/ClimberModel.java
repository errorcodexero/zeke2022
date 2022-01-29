package frc.models;

import java.util.Random;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.simulator.models.SimMotorController;

import edu.wpi.first.hal.SimInt;
import edu.wpi.first.hal.simulation.DIODataJNI;

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
        // of time for the second sensor to hit
        WaitingMidSecondSensor,
    };

    static private final String TouchLeftMidIO = "touch_left_mid_io" ;

    private Random random_;
    private State state_;
    private SimMotorController motor_;
    private SimInt grabber_left_a_;
    private SimInt grabber_left_b_;
    private SimInt grabber_right_a_;
    private SimInt grabber_right_b_;

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

    private double first_sensor_time_ = 0.5;
    private double second_sensor_time_ = 0.5;

    public ClimberModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst);

        random_ = new Random();
        state_ = State.Idle;
    }

    @Override
    public boolean create() {
        MessageLogger logger = getEngine().getMessageLogger() ;
        
        motor_ = new SimMotorController(this, "motor");
        if (!motor_.createMotor())
            return false;

        // if (!hasProperty(TouchLeftMidIO)) {
        //     logger.startMessage(MessageType.Error);
        //     logger.add("event: model ").addQuoted(getModelName());
        //     logger.add(" instance ").addQuoted(getInstanceName());
        //     logger.add(" is missing required property").addQuoted(TouchLeftMidIO);
        //     logger.endMessage();
        //     return false;
        // }
    
        //     if (!hasProperty(IntakeInstancePropertyName)) {
        //         logger.startMessage(MessageType.Error);
        //         logger.add("event: model ").addQuoted(getModelName());
        //         logger.add(" instance ").addQuoted(getInstanceName());
        //         logger.add(" is missing required property").addQuoted(IntakeInstancePropertyName);
        //         logger.endMessage();
        //         return false;
        //     }
    
        //     SettingsValue modelprop = getProperty(IntakeModelPropertyName);

        // touch_left_mid_io_ = getIntProperty(TouchLeftMidIO) ;

        setSensors();

        return true;
    }

    public void run(double dt) {
        switch (state_) {
            case Idle:
                break;

            case Started:
                sensor_side_ = random_.nextInt(2);
                phase_start_time_ = getRobotTime();
                state_ = State.WaitingMidFirstSensor;
                break;

            case WaitingMidFirstSensor:
                if (getRobotTime() - phase_start_time_ > first_sensor_time_) {
                    if (sensor_side_ == 0)
                        touch_left_mid_value_ = true;
                    else
                        touch_right_mid_value_ = true;
                    setSensors();

                    sensor_side_ = 1 - sensor_side_;
                    phase_start_time_ = getRobotTime();
                    state_ = State.WaitingMidFirstSensor;
                }
                break;

            case WaitingMidSecondSensor:
                if (getRobotTime() - phase_start_time_ > second_sensor_time_) {
                    if (sensor_side_ == 0)
                        touch_left_mid_value_ = true;
                    else
                        touch_right_mid_value_ = true;
                    setSensors();

                    state_ = State.WaitingMidFirstSensor;
                }
                break;
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

    private void setSensors() {
        DIODataJNI.setValue(touch_left_mid_io_, touch_left_mid_value_);
        DIODataJNI.setValue(touch_left_traverse_io_, touch_left_traverse_value_);
        DIODataJNI.setValue(touch_right_mid_io_, touch_right_mid_value_);
        DIODataJNI.setValue(touch_right_traverse_io_, touch_right_traverse_value_);
        DIODataJNI.setValue(touch_left_high_io_, touch_left_high_value_);
        DIODataJNI.setValue(touch_right_high_io_, touch_right_high_value_);
    }
}
