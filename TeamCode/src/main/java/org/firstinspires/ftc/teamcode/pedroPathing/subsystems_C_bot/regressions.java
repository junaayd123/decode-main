package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.lifters;


public class regressions {
    lifters LL;
    public int distanceToVelo(double dist){

        //https://www.desmos.com/calculator/nxghj961jg
        if(dist<125){
//            return (int) (7.07643*dist+1142.07867);
            if(dist<90) return (int) (7.3355*dist+1018.2);
            else return (int) (7.3355*dist+1038.2);
        }
        else{
            return (int)(4.49259*(dist)+1600.95157);
        }
    }
    public double distanceToAngle(double dist){
        if (dist>125) return 0.26;//far
        else{
//            if(dist<100) return 0.00194321*dist+0.0488222;
//            else return 0.00194321*dist+0.0388222;
        return 0.001838*dist+0.0434;
        }
    }
}
