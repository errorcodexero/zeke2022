package frc.robot.automodes;

import org.xero1425.base.motorsubsystem.MotorEncoderGotoAction;
import org.xero1425.base.tankdrive.TankDrivePathFollowerAction;
import org.xero1425.base.tankdrive.TankDriveSubsystem;

import frc.robot.conveyor.ConveyorSetBall;
import frc.robot.gpm.GPMFireAction;
import frc.robot.gpm.GPMSubsystem;
import frc.robot.targettracker.TargetTrackerSubsystem;
import frc.robot.turret.TurretSubsystem;
import frc.robot.zekesubsystem.ZekeSubsystem;

public class NearTarmac2BallAuto extends ZekeAutoMode {

    public NearTarmac2BallAuto(ZekeAutoController ctrl, String name) throws Exception {
        super(ctrl, name);

        ZekeSubsystem zeke = (ZekeSubsystem)ctrl.getRobot().getRobotSubsystem() ;
        GPMSubsystem gpm = zeke.getGPMSubsystem() ;
        TankDriveSubsystem db = zeke.getTankDrive() ;
        TargetTrackerSubsystem tracker = zeke.getTargetTracker() ;
        TurretSubsystem turret = zeke.getTurret() ;
        
        addSubActionPair(gpm.getConveyor(), new ConveyorSetBall(gpm.getConveyor()), false);
        driveAndCollect("near_tarmac_2_ball_1", 1.0, 0.0);
        addSubActionPair(db, new TankDrivePathFollowerAction(db, "near_tarmac_2_ball_2", true), true) ;
        addSubActionPair(zeke.getTurret(), new MotorEncoderGotoAction(zeke.getTurret(), 0.0, true), false);
        startTracking();
        addSubActionPair(gpm, new GPMFireAction(gpm, tracker, db, turret), true);
    }
}
