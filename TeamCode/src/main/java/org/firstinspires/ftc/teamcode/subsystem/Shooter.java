package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Shooter extends Subsystem {
    public final static String FLYWHEEL_NAME = "pollemShooter";

    public final static double DEFAULT_TARGET_SPEED = 1480;
    public final static double DEFAULT_SPEED_TOLERANCE = 30.0;
    public final static int ITERATIONS_TO_ASSUME_AT_SPEED = 4;

    private DcMotorEx shooter;

    private double targetSpeed;
    private double speedTolerance;
    private int iterationsForSpeed;

    private ShooterState shooterState;

    public void init(HardwareMap hardwareMap){
        shooter = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(56, 0.8,0,10));
        this.speedTolerance = DEFAULT_SPEED_TOLERANCE;

    }

    public void setSpeedTolerance(double speedTolerance){
        this.speedTolerance = speedTolerance;
    }
    public void writeSpeeds(Telemetry telemetry){
        telemetry.addData("Target Speed", targetSpeed);
        telemetry.addData("Actual Speeds", shooter.getVelocity());
        //telemetry.addData("Right Speed", rightShooter.getVelocity());
    }
    public void setIdleShooter(){
        shooterState = ShooterState.IDLE;
    }
    public void startShooting(){
        startShooting(DEFAULT_TARGET_SPEED);
    }
    public void startShooting(double defaultSpeed){
        shooterState = ShooterState.SPINNING_UP;
        targetSpeed = defaultSpeed;
        doSpinningUp();
    }
    public boolean isReadyForShot(){
        return shooterState == ShooterState.READY_FOR_SHOT;
    }


    @Override
    public void loop(){
        if (shooterState == ShooterState.IDLE) doIdle();
        else if (shooterState == ShooterState.SPINNING_UP) doSpinningUp();
        else if (shooterState == ShooterState.READY_FOR_SHOT) doReadyForShot();
    }

    private void doIdle(){
        shooter.setPower(0.0);
    }
    private void doSpinningUp(){
        computeTargetSpeed();
        if (isAtSpeed()) {
            if (iterationsForSpeed >= ITERATIONS_TO_ASSUME_AT_SPEED) {
                shooterState = ShooterState.READY_FOR_SHOT;
                doReadyForShot();
            }else {
                iterationsForSpeed++;
            }
        }else{
            iterationsForSpeed = 0;
        }
        shooter.setVelocity(targetSpeed);

    }
    private void doReadyForShot(){
        iterationsForSpeed = 0;
        computeTargetSpeed();
        if (!isAtSpeed()) shooterState = ShooterState.SPINNING_UP;
    }

    private void computeTargetSpeed(){
        double cameraSpeed = getIdealShootingSpeed();
        if (cameraSpeed>100){
            targetSpeed = cameraSpeed;
        }
    }
    private boolean isAtSpeed(){
        return isAtSpeed(this.speedTolerance);
    }
    public boolean isAtSpeed(double speedTolerance){
        return shooter.getVelocity() < targetSpeed+ speedTolerance && shooter.getVelocity() > targetSpeed - speedTolerance;
    }
    private double getIdealShootingSpeed()  {
        return targetSpeed;
    }
    public double getTargetSpeed() {return targetSpeed;}
    public void setTargetSpeed(double targetSpeed) {this.targetSpeed = targetSpeed;}

    private enum ShooterState {
        IDLE, SPINNING_UP, READY_FOR_SHOT, MANUAL_CONTROL
    }
}
