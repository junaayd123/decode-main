package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;

public class ShooterManager {
    private Deposition_C depo;
    private lifters lift;
    private regressions reg;
    private Timer timer;
    
    private boolean shooting = false;
    public boolean isBlue = false;
    private int shooterSequence = 0;
    private double shootInterval = 0.35;
    private String sequence = "rbl";
    private boolean motifShot=false;
    public void setMotifShot(boolean m){
        motifShot = m;
    }
    
    public int firstShot, secondShot, thirdShot;

    public ShooterManager(Deposition_C depo, lifters lift, regressions reg) {
        this.depo = depo;
        this.lift = lift;
        this.reg = reg;
        this.timer = new Timer();
    }
    
    public void setShootInterval(double interval) {
        this.shootInterval = interval;
    }
    
    public void updateTarget(double distance, boolean shootingTest, double manualVelo) {
        if(!motifShot) {
            if (distance > 125) {
                shootInterval = 0.25;
            } else {
                shootInterval = 0.2;
            }
        }
        else shootInterval = .5;
//        shootInterval = 0.2;

        if (shootingTest) {
            depo.setTargetVelocity(manualVelo);
        } else {
            if(shooting) {
                if(isBlue){
                    depo.setTargetVelocity(reg.distanceToVeloBlue(distance));
                }
                else {
                    depo.setTargetVelocity(reg.distanceToVelo(distance));
                }
            }
            lift.set_angle_custom(reg.distanceToAngle(distance));
        }
    }
    
    public void startShooting(String motif, int ballOnRamp, int greenInSlot) {
        this.sequence = determineSequence(motif, ballOnRamp, greenInSlot);
        this.shooting = true;
        this.shooterSequence = 0;
    }

    private String determineSequence(String motif, int ballOnRamp, int greenInSlot) {
        if (motif.equals("gpp")) {
            if      ((ballOnRamp==0&&greenInSlot==0)||(ballOnRamp==1&&greenInSlot==2)||(ballOnRamp==2&&greenInSlot==1)) return "lrb";
            else if ((ballOnRamp==0&&greenInSlot==1)||(ballOnRamp==1&&greenInSlot==0)||(ballOnRamp==2&&greenInSlot==2)) return "rbl";
            else return "blr";
        } else if (motif.equals("pgp")) {
            if      ((ballOnRamp==0&&greenInSlot==0)||(ballOnRamp==1&&greenInSlot==2)||(ballOnRamp==2&&greenInSlot==1)) return "blr";
            else if ((ballOnRamp==0&&greenInSlot==1)||(ballOnRamp==1&&greenInSlot==0)||(ballOnRamp==2&&greenInSlot==2)) return "lrb";
            else return "rbl";
        } else { // ppg
            if      ((ballOnRamp==0&&greenInSlot==0)||(ballOnRamp==1&&greenInSlot==2)||(ballOnRamp==2&&greenInSlot==1)) return "rbl";
            else if ((ballOnRamp==0&&greenInSlot==1)||(ballOnRamp==1&&greenInSlot==0)||(ballOnRamp==2&&greenInSlot==2)) return "blr";
            else return "lrb";
        }
    }
    
    public void update() {
        depo.updatePID();
        if (shooting && depo.reachedTargetTeleOP()) {
            timer.startTimer();
            shooting = false;
        }
        
        if (timer.timerIsOn()) {
            if (sequence.equals("lrb")) LRBnoRecovery();
            else if (sequence.equals("rbl")) RBLnoRecovery();
            else if (sequence.equals("blr")) BLRnoRecovery();
        }
    }

    public void stop() {
        timer.stopTimer();
        shooting = false;
        shooterSequence = 0;
        lift.allDown();
        depo.setTargetVelocity(0);
    }
    
    public boolean isShooting() {
        return shooting || timer.timerIsOn();
    }
    
    public int getShooterSequence() {
        return shooterSequence;
    }

    private void LRBnoRecovery() {
        double elapsed = timer.timer.seconds() - timer.curtime;
        if (shooterSequence == 0 && elapsed >= 0) {
            lift.leftUp();
            shooterSequence = 1;
        } else if (shooterSequence == 1 && elapsed >= shootInterval) {
            lift.allDown();
            lift.rightUp();
            shooterSequence = 2;
        } else if (shooterSequence == 2 && elapsed >= shootInterval * 2) {
            lift.allDown();
            lift.backUp();
            shooterSequence = 3;
        } else if (shooterSequence == 3 && elapsed >= shootInterval * 3 + 0.25) {
            lift.allDown();
            depo.setTargetVelocity(0);
            timer.stopTimer();
            shooterSequence = 0;
            motifShot = false;
        }
    }

    private void BLRnoRecovery() {
        double elapsed = timer.timer.seconds() - timer.curtime;
        if (shooterSequence == 0 && elapsed >= 0) {
            lift.backUp();
            shooterSequence = 1;
        } else if (shooterSequence == 1 && elapsed >= shootInterval) {
            lift.allDown();
            lift.leftUp();
            shooterSequence = 2;
        } else if (shooterSequence == 2 && elapsed >= shootInterval * 2) {
            lift.allDown();
            lift.rightUp();
            shooterSequence = 3;
        } else if (shooterSequence == 3 && elapsed >= shootInterval * 3 + 0.25) {
            lift.allDown();
            depo.setTargetVelocity(0);
            timer.stopTimer();
            shooterSequence = 0;
            motifShot = false;
        }
    }

    private void RBLnoRecovery() {
        double elapsed = timer.timer.seconds() - timer.curtime;
        if (shooterSequence == 0 && elapsed >= 0) {
            lift.rightUp();
            shooterSequence = 1;
        } else if (shooterSequence == 1 && elapsed >= shootInterval) {
            lift.allDown();
            lift.backUp();
            shooterSequence = 2;
        } else if (shooterSequence == 2 && elapsed >= shootInterval * 2) {
            lift.allDown();
            lift.leftUp();
            shooterSequence = 3;
        } else if (shooterSequence == 3 && elapsed >= shootInterval * 3 + 0.25) {
            lift.allDown();
            depo.setTargetVelocity(0);
            timer.stopTimer();
            shooterSequence = 0;
            motifShot = false;
        }
    }
}
