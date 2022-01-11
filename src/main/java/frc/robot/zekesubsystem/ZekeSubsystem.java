package frc.robot.zekesubsystem; //directory path from "java"

import org.xero1425.base.RobotSubsystem;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.tankdrive.TankDriveSubsystem;

import frc.robot.zekeoi.ZekeOISubsystem;

public class ZekeSubsystem extends RobotSubsystem {
    public final static String SubsystemName = "bunnybot" ;
    public final static String TankdriveSubsystemName = "tankdrive" ;
    private TankDriveSubsystem db_ ;
    private ZekeOISubsystem oi_;

    public ZekeSubsystem(XeroRobot robot) throws Exception {
        super(robot, SubsystemName) ;

        db_ = new TankDriveSubsystem(this, TankdriveSubsystemName, "tankdrive") ;
        addChild(db_) ;

        oi_ = new ZekeOISubsystem(this, db_) ;
        addChild(oi_) ;
    }

    public TankDriveSubsystem getTankDrive() {
        return db_ ;
    }

    public ZekeOISubsystem getOI() {
        return oi_ ;
    }

}