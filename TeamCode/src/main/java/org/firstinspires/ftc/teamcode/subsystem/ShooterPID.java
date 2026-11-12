package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterPID extends Subsystem {
    public final static String FLYWHEEL_NAME = "pollenShooter";

    public final static double DEFAULT_TARGET_SPEED = 1440;
    public final static double DEFAULT_SPEED_TOLERANCE = 40.0;
    public final static int ITERATIONS_TO_ASSUME_AT_SPEED = 6;

    private DcMotorEx shooter;

    private double targetSpeed;
    private double speedTolerance;
    private int iterationsForSpeed;

    private ShooterState shooterState;

    private double p=15;
    private double i = 0.0072;
    private double d = 0.0178;
    private double f = 13;


    private double integralSum = 0;
    private double lastError = 0;
    private double maxIntegral = 1.0; // Prevents integral windup

    private final ElapsedTime timer = new ElapsedTime();
    private boolean firstRun = true;
    private final double nominalVoltage = 12.0;

    private VoltageSensor batteryVoltageSensor;

    public void init(HardwareMap hardwareMap){
        shooter = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(56, 0.8,0,10));
        this.speedTolerance = DEFAULT_SPEED_TOLERANCE;
        this.shooterState = ShooterState.IDLE;

    }

    public void setSpeedTolerance(double speedTolerance){
        this.speedTolerance = speedTolerance;
    }
    public void writeSpeeds(Telemetry telemetry){
        telemetry.addData("Shooter State", shooterState.name());
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
        setOutput();
    }
    private void setOutput(){
        if (shooterState == ShooterState.IDLE){
            shooter.setPower(0.0); return;
        }
        double current = shooter.getVelocity();
        double error = targetSpeed - current;

        if (firstRun) {
            timer.reset();
            lastError = error;
            firstRun = false;
        }

        double dt = timer.seconds();
        timer.reset();

        // Guard against zero dt on ultra-fast loops
        if (dt <= 0) {
            dt = 1e-6;
        }

        // 1. Proportional term
        double pTerm = p * error;

        // 2. Integral term with anti-windup clamping
        integralSum += error * dt;
        integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);
        double iTerm = i * integralSum;

        // 3. Derivative term
        double derivative = (error - lastError) / dt;
        double dTerm = d * derivative;

        // 4. Feedforward term (scales directly with target setpoint)
        double fTerm = f * targetSpeed;

        lastError = error;

        // Raw output assumes a standard 12.0V battery
        double rawOutput = pTerm + iTerm + dTerm + fTerm;

        // Scale output based on actual battery voltage
        // If battery sags to 10V, power increases by 12/10 (1.2x) to compensate
        double currentVoltage = batteryVoltageSensor.getVoltage();
        double compensatedPower = rawOutput * (nominalVoltage / currentVoltage);

        shooter.setPower(Range.clip(compensatedPower, -1.0, 1.0));
    }


    private void doIdle(){
        shooter.setPower(0.0);
    }
    private void doSpinningUp(){
        computeTargetSpeed();
        if (isAtSpeed()) {
            if (iterationsForSpeed >= ITERATIONS_TO_ASSUME_AT_SPEED) {
                shooterState = ShooterState.READY_FOR_SHOT;
                iterationsForSpeed = 0;
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
        computeTargetSpeed();
        if (!isAtSpeed()){
            if(iterationsForSpeed > ITERATIONS_TO_ASSUME_AT_SPEED) {
                shooterState = ShooterState.SPINNING_UP;
                iterationsForSpeed = 0;
            }else{
                iterationsForSpeed++;
            }
        }else{
            iterationsForSpeed=0;
        }
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

    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        shooter.setVelocityPIDFCoefficients(p,i,d,f);
    }

    private enum ShooterState {
        IDLE, SPINNING_UP, READY_FOR_SHOT, MANUAL_CONTROL
    }
}
