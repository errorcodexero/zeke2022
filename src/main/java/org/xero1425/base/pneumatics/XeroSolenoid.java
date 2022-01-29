package org.xero1425.base.pneumatics;

import org.xero1425.base.Subsystem;
import org.xero1425.base.XeroRobot;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.wpilibj.Solenoid;

public class XeroSolenoid {
    private Solenoid solenoid_ ;
    private SimDevice simdev_ ;
    private SimBoolean simstate_ ;
    private boolean state_ ;

    final private String SimDeviceName = "Solenoid" ;
    final private String SimeStateName = "state" ;

    public XeroSolenoid(XeroRobot robot, int module, int channel) {
        if (XeroRobot.isSimulation()) {
            simdev_ = SimDevice.create(SimDeviceName, calcIndex(module, channel)) ;
            simstate_ = simdev_.createBoolean(SimeStateName, SimDevice.Direction.kBidir, false) ;
        }
        else {
            solenoid_ = new Solenoid(module, robot.getPneumaticsType(), channel) ;
        }
    }

    public XeroSolenoid(Subsystem sub, String name) {
    }

    public XeroSolenoid(XeroRobot robot, int channel) {
        if (XeroRobot.isSimulation()) {
            simdev_ = SimDevice.create(SimDeviceName, calcIndex(0, channel)) ;
            simstate_ = simdev_.createBoolean(SimeStateName, SimDevice.Direction.kBidir, false) ;
        }
        else {
            solenoid_ = new Solenoid(robot.getPneumaticsType(), channel) ;
        }
    }
    
    public void close() {
        if (!XeroRobot.isSimulation()) {
            solenoid_.close() ;
        }
    }

    public void set(boolean on) {
        state_ = on ;

        if (XeroRobot.isSimulation()) {
            simstate_.set(on);
        }
        else {
            solenoid_.set(on) ;
        }
    }

    public boolean get() {
        return state_ ;
    }

    public void toggle() {
        set(!state_) ;
    }

    public boolean isDisabled() {
        boolean ret = false ;

        if (!XeroRobot.isSimulation()) {
            ret = solenoid_.isDisabled() ;
        }

        return ret ;
    }

    private int calcIndex(int module, int channel) {
        return module * 32 + channel ;
    }
}
