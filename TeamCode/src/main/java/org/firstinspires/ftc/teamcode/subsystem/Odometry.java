package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

import java.util.Locale;

public class Odometry extends Subsystem {
    private Telemetry telemetry;

    public GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer


    public void init(HardwareMap hardwareMap, Telemetry telemetry, Pose2D initialPose){
        this.telemetry = telemetry;
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(84.0, 0.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setPosition(initialPose);
    }

   /*@Override
    public void playOnceImpl(){
        //odo.resetPosAndIMU();
        if (telemetry == null) return;
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
    }
*/
    public void setOdoPositions(double x, double y, double heading){
        odo.setPosition( new Pose2D(DistanceUnit.MM,x, y, AngleUnit.DEGREES, heading));
    }

    public Pose2D getOdoPosition(){
        return odo.getPosition();
    }
    public double getHeading(){
        return odo.getHeading(AngleUnit.DEGREES);
    }

    public void resetPosition(){
        odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
    }
    public void recalibrateIMU(){
        odo.recalibrateIMU(); //recalibrates the IMU without resetting position
    }

    @Override
    public void loop(){
        odo.update();
     }
    public void writeOutPosition(Telemetry telemetry){
        odo.update();
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

    }
}
