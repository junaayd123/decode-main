package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Timer {
    public ElapsedTime timer = new ElapsedTime();
    public double curtime = 10000;
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
        if(timer.seconds()>=curtime+sec && timer.seconds()<curtime+sec+0.05) return true;
        else return false;
    }
    public boolean timerIsOn(){
        if(curtime ==10000) return false;
        else return true;
    }

}
