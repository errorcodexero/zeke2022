package frc.robot.zekesubsystem; //directory path from "java"

import org.xero1425.base.RobotSubsystem;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.tankdrive.TankDriveSubsystem;

import frc.robot.gpm.GPMSubsystem;
import frc.robot.zekeoi.ZekeOISubsystem;

public class ZekeSubsystem extends RobotSubsystem {
    public final static String SubsystemName = "zeke" ;
    public final static String TankdriveSubsystemName = "tankdrive" ;
    private TankDriveSubsystem db_ ;
    private ZekeOISubsystem oi_;
    private GPMSubsystem gpm_ ;

    public ZekeSubsystem(XeroRobot robot) throws Exception {
        super(robot, SubsystemName) ;

        db_ = new TankDriveSubsystem(this, TankdriveSubsystemName, "tankdrive") ;
        addChild(db_) ;

        oi_ = new ZekeOISubsystem(this, db_) ;
        addChild(oi_) ;

        gpm_ = new GPMSubsystem(this, db_) ;
    }

    public TankDriveSubsystem getTankDrive() {
        return db_ ;
    }

    public ZekeOISubsystem getOI() {
        return oi_ ;
    }

}