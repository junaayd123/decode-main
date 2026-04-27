package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.lifters;


public class regressions {
    lifters LL;
    public double RobY;
    public double RobHeading;

    public int distanceToVelo(double dist) {
        //https://www.desmos.com/calculator/nxghj961jg
        if(RobY==0) {
            if (dist < 125) { //close
                return (int) (7.3355 * dist + 1038.2+20);
            } else {
                return (int) (9.36897 * dist + 771.99703-20);
            }
        }
        else{
            if (RobY > 55) { //close
                return (int) (7.3355 * dist + 1038.2+20);
            } else {
                return (int) (9.36897 * dist + 771.99703-20);
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

    public double getRedTurretFar(double X, double Y, double H) {
        double Xreg = -0.484121 * X - 69;
        double Yreg = 0.18 * Y - 1.22;
        double Hreg =  -0.0055555*3 * Math.abs(Math.toDegrees(H));
        return Xreg + Yreg+Hreg;
    }
    public double getBlueTurretFar(double X, double Y) {
        double Xreg = -0.484121 * X - 115;
        double Yreg = -0.112277 * Y - 1.22382;
        return Xreg + Yreg;
    }
}
