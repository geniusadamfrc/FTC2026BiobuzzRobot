
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.ShooterPID;


@TeleOp(name="Shooter Tuner", group="Test")
//@Disabled
public class ShooterTuner extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private Shooter shooter = null;
    //private DcMotorEx shooter2 = null;
    private double speed;
    private double p=15;
    private double i = 0.0072;
    private double d = 0.0178;
    private double f = 13;

    private STATE state;
    public static final double INC = 1.01;

    @Override
    public void runOpMode() {
        shooter = new Shooter();
        shooter.init(hardwareMap);
        speed = 1480;

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Speed", speed);
        telemetry.update();

        state = STATE.SPEED;
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {






            if (gamepad1.a){ shooter.startShooting(speed);}
            else if (gamepad1.b) shooter.setIdleShooter();

            if (gamepad1.left_bumper){
                state = STATE.P;
            }else if (gamepad1.right_bumper) state = STATE.I;
            else if (gamepad1.x) state = STATE.D;
            else if (gamepad1.y) state = STATE.F;
            else if (gamepad1.right_trigger_pressed) state = STATE.SPEED;

            if (gamepad1.dpad_up){
                if (state == STATE.P){
                    p*=INC;
                }else if (state == STATE.I) {
                    i*=INC;
                }else if (state == STATE.D) {
                    d*=INC;
                }else if (state == STATE.F) {
                    f*=INC;
                }else if (state == STATE.SPEED) {
                    speed*=INC;
                }
            }else if (gamepad1.dpad_down){
                if (state == STATE.P){
                    p/=INC;
                }else if (state == STATE.I) {
                    i/=INC;
                }else if (state == STATE.D) {
                    d/=INC;
                }else if (state == STATE.F) {
                    f/=INC;
                }else if (state == STATE.SPEED) {
                    speed/=INC;
                }
            }



            shooter.setVelocityPIDFCoefficients(p,i,d,f);
            shooter.loop();
            // Show the elapsed game time and wheel power.
            telemetry.addData("Tuning", state.name());
            telemetry.addData("Target speed",speed);
            shooter.writeSpeeds(telemetry);
            telemetry.addData("P", p);
            telemetry.addData("I", i);
            telemetry.addData("D", d);
            telemetry.addData("F", f);
            telemetry.update();
        }
    }

    public enum STATE {
        P, I, D, F, SPEED;
    }
}
