package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class TurretLimelight {
    public DcMotorEx TurretMotor;   // Motor port used to read encoder

    private PIDController pid;
    public static double p = 0.005;
    public static double i = 0.1;
    public static double d = 0.0001;
    public static int target = 0;
    public static double tolerance = 1.0;
    public static double turetSpeed = 0.8;
    public static double targetDegrees = 0.0;
    double coefficient = 67.0/18.0;
    private Limelight3A limelight;
    public double yawToTag;
    boolean blueAlliance;
    double targetTicks;
    boolean runningAround;
    boolean doneRUnning;
    double groundDistanceCM;
    double power;
    boolean hasTag;
    public double currentPos;
    public TurretLimelight(HardwareMap hardwareMap) {
        TurretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        pid = new PIDController(p, i, d);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }
    public void resetTurretEncoder(){
        TurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void InitLimelight(){
        limelight.pipelineSwitch(0);
        limelight.start();
    }
    public void updateLimelight(){
        pid.setPID(p, i, d);

        currentPos = TurretMotor.getCurrentPosition();

        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        yawToTag = 0;
        hasTag = false;
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            if(blueAlliance){
                if (fr.getFiducialId() == 20) {
                    yawToTag = fr.getTargetXDegrees();
                    hasTag = true;
                    Pose3D camToTag = fr.getCameraPoseTargetSpace();
                    groundDistanceCM = Math.hypot(camToTag.getPosition().x, camToTag.getPosition().z)*100;
                }
                else{
                    yawToTag = 0;
                    hasTag = false;
                }
            }
            else {
                if (fr.getFiducialId() == 24) {
                    yawToTag = fr.getTargetXDegrees();
                    hasTag = true;
                    Pose3D camToTag = fr.getCameraPoseTargetSpace();
                    groundDistanceCM = Math.hypot(camToTag.getPosition().x, camToTag.getPosition().z)*100;
                }
                else{
                    yawToTag = 0;
                    hasTag = false;
                }
            }

        }
    }
    public void toTargetInTicks(){
        double error = target - currentPos;
        if (Math.abs(error) <= tolerance) {
            power = 0.0;
        } else {
            double pidOutput = pid.calculate(currentPos, target);
            power = Math.max(-turetSpeed, Math.min(turetSpeed, pidOutput));
        }
        TurretMotor.setPower(power);
    }
    public void setTicksTarget(int targett){
        target = targett;
    }
    public void toTargetInDegrees(){
        // PID control
        double targetTicks = targetDegrees*coefficient;
        double error = targetTicks - currentPos;

        if (Math.abs(error) <= tolerance) {
            power = 0.0;
        } else {
            double pidOutput = pid.calculate(currentPos, targetTicks);
            power = Math.max(-turetSpeed, Math.min(turetSpeed, pidOutput));
        }
        TurretMotor.setPower(power);
    }
    public void setDegreesTarget(double deg){
        targetDegrees = deg;
    }
    public void allignToTag(){
        if(!runningAround) {
            if(groundDistanceCM > 140){//for farshot make it face 2.5 degrees toward the outside of the tag
                if(blueAlliance) targetTicks = currentPos + ((yawToTag-2.5) * coefficient);
                else targetTicks = currentPos + ((yawToTag+2.5) * coefficient);
            }
            else targetTicks = currentPos + (yawToTag * coefficient);
        }
        if(targetTicks>730){
            targetTicks-=1300;
            runningAround = true;
            doneRUnning = false;
        }
        else if(targetTicks<-850){
            targetTicks+=1300;
            runningAround = true;
            doneRUnning = false;
        }
        else {
            if(doneRUnning) {
                runningAround = false;
            }
        }
        double error = targetTicks - currentPos;
        if(Math.abs(error)<=3 && runningAround){
            doneRUnning = true;
        }
        if (Math.abs(error) <= tolerance ||(!hasTag && !runningAround)) {
            power = 0.0;
        } else {
            double pidOutput = pid.calculate(currentPos, targetTicks);
            power = Math.max(-turetSpeed, Math.min(turetSpeed, pidOutput));
        }

            TurretMotor.setPower(power);
    }
    public void setRedAlliance(){ blueAlliance = false;}
    public void setBlueAlliance(){ blueAlliance = true;}
    public double getTagDistance(){
        return groundDistanceCM;
    }


}
