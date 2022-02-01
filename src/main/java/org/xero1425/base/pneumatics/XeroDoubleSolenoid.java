package org.xero1425.base.pneumatics;

import org.xero1425.base.Subsystem;
import org.xero1425.base.XeroRobot;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class XeroDoubleSolenoid {
    private DoubleSolenoid solenoid_ ;
    private SimDevice simdev_ ;
    private SimInt simstate_ ;
    private DoubleSolenoid.Value state_ ;
    private int index_ ;

    static final public String SimDeviceName = "DoubleSolenoid" ;
    static final public String SimeStateName = "state" ;

    public XeroDoubleSolenoid(XeroRobot robot, int module, int forward, int reverse) {
        if (XeroRobot.isSimulation()) {
            index_ = calcIndex(module, forward) ;
            simdev_ = SimDevice.create(SimDeviceName, index_) ;
            simstate_ = simdev_.createInt(SimeStateName, SimDevice.Direction.kBidir, 0) ;
        }
        else {
            solenoid_ = new DoubleSolenoid(module, robot.getPneumaticsType(), forward, reverse) ;
        }

        state_ = DoubleSolenoid.Value.kOff ;
    }

    public XeroDoubleSolenoid(XeroRobot robot, int forward, int reverse) {
        if (XeroRobot.isSimulation()) {
            index_ = calcIndex(0, forward) ;
            simdev_ = SimDevice.create(SimDeviceName, index_) ;
            simstate_ = simdev_.createInt(SimeStateName, SimDevice.Direction.kBidir, 0) ;
        }
        else {
            solenoid_ = new DoubleSolenoid(robot.getPneumaticsType(), forward, reverse) ;
        }
    }

    public XeroDoubleSolenoid(Subsystem sub, String name) throws BadParameterTypeException, MissingParameterException {
        this(sub.getRobot(), sub.getSettingsValue("hw:solenoids:" + name + ":forward").getInteger(),
        sub.getSettingsValue("hw:solenoids:" + name + ":reverse").getInteger()) ; 
    }

    public XeroDoubleSolenoid(Subsystem sub, int module, String name) throws BadParameterTypeException, MissingParameterException {
        this(sub.getRobot(), module, 
                sub.getSettingsValue("hw:solenoids:" + name + ":forward").getInteger(),
                sub.getSettingsValue("hw:solenoids:" + name + ":reverse").getInteger()) ; 
    }

    public void close() {
        if (!XeroRobot.isSimulation()) {
            solenoid_.close() ;
        }
    }

    public void set(DoubleSolenoid.Value value) {
        state_ = value ;

        if (XeroRobot.isSimulation()) {
            simstate_.set(value.ordinal());
        }
        else {
            solenoid_.set(value) ;
        }
    }

    public DoubleSolenoid.Value get() {
        return state_ ;
    }

    public void toggle() {
        if (state_ == DoubleSolenoid.Value.kForward) {
            set(DoubleSolenoid.Value.kReverse) ;
        }
        else if (state_ == DoubleSolenoid.Value.kReverse) {
            set(DoubleSolenoid.Value.kForward) ;
        }
    }

    public boolean isFwdSolenoidDisabled() {
        boolean ret = false ;

        if (!XeroRobot.isSimulation()) {
            ret = solenoid_.isFwdSolenoidDisabled() ;
        }

        return ret ;
    }

    public boolean isRevSolenoidDisabled() {
        boolean ret = false ;

        if (!XeroRobot.isSimulation()) {
            ret = solenoid_.isRevSolenoidDisabled() ;
        }

        return ret ;
    }

    static public int calcIndex(int module, int channel) {
        return module * 32 + channel ;
    }
}
