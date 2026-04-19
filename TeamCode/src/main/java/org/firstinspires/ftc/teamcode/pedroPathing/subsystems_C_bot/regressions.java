package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.lifters;


public class regressions {
    lifters LL;
    public int distanceToVelo(double dist){

        //https://www.desmos.com/calculator/nxghj961jg
        if(dist<125){
            return (int) (7.3355*dist+1038.2);
//            if(dist<80) return (int) (7.3355*dist+1018.2);
//            else return (int) (7.3355*dist+1038.2);
        }
        else{
//            return (int)(4.49259*(dist)+1600.95157);
            return (int)(9.36897*dist+771.99703);
        }
    }
    public double distanceToAngle(double dist){
        if (dist>125) {
//            return 0.26;//far
            return 0.000794358*dist+0.0885449;
        }
        else{
//            if(dist<100) return 0.00194321*dist+0.0488222;
//            else return 0.00194321*dist+0.0388222;
        return 0.001838*dist+0.0434;
        }
    }

    public double getRedTurretFar(double RobX,double RobY) {
        double Xreg = -0.484121*RobX-67.56825;
        double Yreg = 0.112277*RobY-1.22382;
        return Xreg+Yreg;
    }
}
