package org.firstinspires.ftc.teamcode.subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Drivetrain extends Subsystem {
    public static final String leftFrontName = "leftFront";
    public static final String rightFrontName = "rightFront";
    public static final String leftBackName = "leftBack";
    public static final String rightBackName = "rightBack";
    public DcMotorEx leftFrontDrive   = null;
    public DcMotorEx  rightFrontDrive  = null;
    public DcMotorEx  leftBackDrive   = null;
    public DcMotorEx  rightBackDrive  = null;


    public void init(HardwareMap hardwareMap){
        initMotors(hardwareMap);
    }
    private void initMotors(HardwareMap hardwareMap){


        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, leftFrontName); //0
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, rightFrontName); //3
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, leftBackName); //1
        rightBackDrive = hardwareMap.get(DcMotorEx.class, rightBackName); //2

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
    }





    public void setBrakeMode(){
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setCoastMode(){
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void driveRobotRelative(double forward, double turn, double strafe){
        double fls = forward + turn + strafe;
        double frs = forward - turn - strafe;
        double bls = forward + turn - strafe;
        double brs = forward - turn + strafe;
         
        double maxSpeed = computeMaxSpeed(fls, frs, bls, brs);
        double minSpeed = computeMinSpeed(fls, frs, bls, brs);
        maxSpeed = Math.max(Math.abs(maxSpeed), Math.abs(minSpeed));
        
        if(maxSpeed>1){
            fls = fls/maxSpeed;
            frs = frs/maxSpeed;
            bls = bls/maxSpeed;
            brs /= maxSpeed;
        }
        setPowersRaw(fls, frs, bls,brs);
        
    }
    public void driveFieldRelative(double y, double x, double rx, double botHeading){

// 3. Rotate the movement vector based on the robot's heading
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

// Denominator ensures power values never exceed 1.0
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double fls = (rotY + rotX + rx) / denominator;
        double bls  = (rotY - rotX + rx) / denominator;
        double frs = (rotY - rotX - rx) / denominator;
        double brs  = (rotY + rotX - rx) / denominator;
        setPowersRaw(fls, frs, bls,brs);
    }

    private double computeMaxSpeed(double fls, double frs, double bls, double brs){
        double maxSpeed;
        maxSpeed = Math.max(frs, fls);
        maxSpeed = Math.max(maxSpeed, bls);
        maxSpeed = Math.max(maxSpeed, brs);
        return maxSpeed;
    }
    private double computeMinSpeed(double fls, double frs, double bls, double brs){
        double minSpeed = Math.min(frs, fls);
        minSpeed = Math.min(minSpeed, bls);
        minSpeed = Math.min(minSpeed, brs);
        return minSpeed;
    }

    public void setPowersRaw(double fls, double frs, double bls, double brs){
        leftFrontDrive.setPower(fls);
        rightFrontDrive.setPower(frs);
        leftBackDrive.setPower(bls);
        rightBackDrive.setPower(brs);

    }
    public void setPowersRaw(double fls, double frs, double bls, double brs, double threshold){
        if(Math.abs(leftFrontDrive.getPower() -fls) > threshold) leftFrontDrive.setPower(fls);
        if(Math.abs(leftBackDrive.getPower() -bls) > threshold) leftBackDrive.setPower(bls);
        if(Math.abs(rightFrontDrive.getPower() -frs) > threshold) rightFrontDrive.setPower(frs);
        if(Math.abs(rightBackDrive.getPower() -brs) > threshold) rightBackDrive.setPower(brs);
    }




    //@Override
    public void loop2(Telemetry telemetry) {
        //telemetry.addData("Out", aligner.out);
    }








}