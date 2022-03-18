package frc.robot.climber;

public class ClimbPowerController {
    private double target_ ;
    private double max_power_ ;
    private double finish_power_ ;
    private double finish_range_ ;
    private double start_range_ ;

    private int loops_ ;

    public ClimbPowerController(double target, double startrange, double maxpower, double finishpower, double finishrange) {
        target_ = target ;
        max_power_ = maxpower ;
        finish_power_ = finishpower ;
        finish_range_ = finishrange ;
        start_range_ = startrange ;
    }

    public void start() {
        loops_ = 0 ;
    }

    public double getOutput(double position) {
        double out = 0.0 ;

        double deltatarget = Math.abs(position - target_) ;

        loops_++ ;

        if (loops_ < start_range_) {
            out = loops_ * max_power_ / start_range_ ;
        }
        else if (deltatarget < finish_range_) {
            out = finish_power_ ;
        }
        else {
            out = max_power_ ;
        }

        return out ;
    }
}
