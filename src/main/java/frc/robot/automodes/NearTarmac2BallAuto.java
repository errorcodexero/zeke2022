package frc.robot.automodes;

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
        TargetTrackerSubsystem tracker = zeke.getTargetTracker() ;
        TankDriveSubsystem db = zeke.getTankDrive() ;
        TurretSubsystem turret = zeke.getTurret() ;
        
        startTracking() ;
        addSubActionPair(gpm.getConveyor(), new ConveyorSetBall(gpm.getConveyor()), false);
        driveAndCollect("near_tarmac_2_ball_1", 1.0, 0.0);
        addSubActionPair(gpm, new GPMFireAction(gpm, tracker, db, turret), true);
    }
}
