package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Timer {
    public ElapsedTime timer = new ElapsedTime();
    public double curtime = 10000;
    public double forwardTolerance = 0.02;
    public double tolerance = 0.06;
    public Timer(){
        timer = new ElapsedTime();

    }
    public void resetTimer(){
        timer.reset();
    }
    public void startTimer(){
        curtime = timer.seconds();
    }
    public void stopTimer(){
        curtime = 10000;
    }
    public boolean checkAtSeconds(double sec){
        if(timer.seconds()>=curtime+sec-forwardTolerance && timer.seconds()<curtime+sec+tolerance) return true;
        else return false;
    }
    //mainly going to use this for shooting sequences on the B-bot, i made this cuz loop times take too long and its horrible
    public boolean checkAtSecondsTolerance(double sec, double afterTolerance){
        if(timer.seconds()>=curtime+sec-forwardTolerance && timer.seconds()<curtime+sec+afterTolerance) return true;
        else return false;
    }

    public boolean timerIsOn(){
        if(curtime ==10000) return false;
        else return true;
    }
    public boolean timerIsOff(){
        if(curtime ==10000) return true;
        else return false;
    }

}
