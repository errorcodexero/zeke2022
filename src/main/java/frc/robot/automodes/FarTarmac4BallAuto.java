package frc.robot.automodes;

public class FarTarmac4BallAuto extends ZekeAutoMode {

    public FarTarmac4BallAuto(ZekeAutoController ctrl, String name) throws Exception {
        super(ctrl, name);
        driveAndCollect("far_tarmac_4_ball_1", 2, 0.0);
        driveAndFire("far_tarmac_4_ball_2", true, 0.0);
        driveAndCollect("far_tarmac_4_ball_3", 2, 0.0);
        driveAndFire("far_tarmac_4_ball_4", true, 0.0);

    }

    
}
