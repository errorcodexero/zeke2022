package frc.robot.automodes;

public class FarTarmac2BallAuto extends ZekeAutoMode {

    public FarTarmac2BallAuto(ZekeAutoController ctrl, String name) throws Exception {
        super(ctrl, name);
        driveAndCollect("far_tarmac_2_ball_1", 2, 0.0);
        driveAndFire("far_tarmac_2_ball_2", true, 0.0);
    }
    
}
