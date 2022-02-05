package frc.robot.automodes;

public class NearTarmac2BallAuto extends ZekeAutoMode {

    public NearTarmac2BallAuto(ZekeAutoController ctrl, String name) throws Exception {
        super(ctrl, name);
        driveAndCollect("near_tarmac_2_ball_1", 2, 0.0);
        driveAndFire("near_tarmac_2_ball_2", true, 0.0);
    }
    
}
