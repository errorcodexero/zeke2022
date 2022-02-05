package frc.robot.conveyor;

import org.xero1425.base.Subsystem;
import org.xero1425.base.motors.MotorController;
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
        OPEN_EXIT,
        WAIT_EXIT1,
        WAIT_EXIT0,
        START_UPTAKE,
        WAIT_CHIMNEY1,
        WAIT_CHIMNEY0,
        STOP_UPTAKE,
    }
    private static final DoubleSolenoid.Value ExitCloseState = DoubleSolenoid.Value.kForward ;
    private static final DoubleSolenoid.Value ExitOpenState = DoubleSolenoid.Value.kReverse ;
    public static final int MAX_BALLS = 2;
    private int ball_count_ ;                       // The number of balls stored in the conveyor
    private CargoType[] ball_types_;
    private MotorController intake_motor_ ;         // The motor for the flash intake piece
    private MotorController shooter_motor_ ;        // The motor for the shooter sidef of the conveyor
    private XeroDoubleSolenoid exit_ ;    
    private ZekeColorSensor color_sensor_;
    private static final int SENSOR_COUNT = 4;
    private static final int SENSOR_IDX_INTAKE = 0;
    private static final int SENSOR_IDX_EXIT = 1;
    private static final int SENSOR_IDX_CHIMNEY = 2;
    private static final int SENSOR_IDX_SHOOTER = 3;
    private State state_;


    private DigitalInput[] sensors_; // The array of ball detect sensors
    private boolean[] sensor_states_; // The states of ball detect sensors
    private boolean[] prev_sensor_states_; // The states of ball detect sensors last robot loop
    
    public static final String SubsystemName = "conveyor";
    public static final String SensorLoggerName = "conveyor:sensors:messages";


    public ConveyorSubsystem(Subsystem parent, ZekeColorSensor color) throws Exception {
        super(parent, SubsystemName);
      
        state_ = State.WAIT_INTAKE;
        color_sensor_ = color;
        exit_ = new XeroDoubleSolenoid(this, "exit") ;

        sensors_ = new DigitalInput[SENSOR_COUNT];
        sensor_states_ = new boolean[SENSOR_COUNT];
        prev_sensor_states_ = new boolean[SENSOR_COUNT];

        ball_count_ = 0 ;
        ball_types_ = new CargoType [MAX_BALLS];
        for (int i = 0; i < MAX_BALLS; i++) {
            ball_types_ [i]= CargoType.None;
        }
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
            prev_sensor_states_[i] = sensor_states_[i];
            sensor_states_[i] = !sensors_[i].get();
        }
       CargoType cargoType =  color_sensor_.getCargoType(color_sensor_.getConveyorIndex());

        switch (state_) {
           case WAIT_INTAKE:
               if (prev_sensor_states_[SENSOR_IDX_INTAKE]==false && sensor_states_[SENSOR_IDX_INTAKE]==true)
                   state_ = State.WAIT_COLOR;
               break;
           case  WAIT_COLOR:
                if (cargoType == CargoType.Opposite)
                    state_ = State.OPEN_EXIT;
                else if (cargoType == CargoType.Same)
                    state_ = State.START_UPTAKE;
               break;
           case  OPEN_EXIT:
               exit_.set(ExitOpenState);
               state_ = State.WAIT_EXIT1;
               break;
           case  WAIT_EXIT1:
               if (sensor_states_[SENSOR_IDX_EXIT]==true)
                   state_ = State.WAIT_EXIT0;
               break;
           case  WAIT_EXIT0:
               break;
           case  START_UPTAKE:
               break;
           case  WAIT_CHIMNEY1:
               break;
           case  WAIT_CHIMNEY0:
               break;
           case  STOP_UPTAKE:
               break;
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

            intake_motor_.set(intake) ;
            shooter_motor_.set(shooter) ;
        }
        catch(Exception ex) {
        }
    }  
} ;