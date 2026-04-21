package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.lifters;


public class regressions {
    lifters LL;
    public double RobY;

    public int distanceToVelo(double dist) {
        //https://www.desmos.com/calculator/nxghj961jg
        if(RobY==0) {
            if (dist < 125) { //close
                return (int) (7.3355 * dist + 1038.2);
            } else {
                return (int) (9.36897 * dist + 771.99703);
            }
        }
        else{
            if (RobY > 55) { //close
                return (int) (7.3355 * dist + 1038.2);
            } else {
                return (int) (9.36897 * dist + 771.99703);
            }
        }
    }

    public double distanceToAngle(double dist) {
        if (RobY == 0) {

            if (dist > 125) { //far
                return 0.000794358 * dist + 0.0885449;
            } else {
                return 0.001838 * dist + 0.0434;
            }
        }
        else{
            if (RobY<55) { //far
                return 0.000794358 * dist + 0.0885449;
            } else {
                return 0.001838 * dist + 0.0434;
            }
        }
    }

    public double getRedTurretFar(double RobX, double RobY) {
        double Xreg = -0.484121 * RobX - 67.56825;
        double Yreg = 0.112277 * RobY - 1.22382;
        return Xreg + Yreg;
    }
    public double getBlueTurretFar(double RobX, double RobY) {
        double Xreg = -0.484121 * RobX - 115;
        double Yreg = -0.112277 * RobY - 1.22382;
        return Xreg + Yreg;
    }
}
