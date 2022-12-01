package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "teleop2022")

// INITILIZATION

public class telelop2022 extends LinearOpMode {
    //Decalring Devices
    private DcMotor rf; //port 0
    private DcMotor lf; //port 0
    private DcMotor rb; //port 0
    private DcMotor lb; //port 0
    private Servo wrist;
    private Servo fingers;

    @Override

    public void runOpMode() {
//Mapping Devices
        rf = hardwareMap.dcMotor.get("rightFront");
        lf = hardwareMap.dcMotor.get("leftFront");
        rb = hardwareMap.dcMotor.get("rightBack");
        lb = hardwareMap.dcMotor.get("leftBack");
        wrist = hardwareMap.servo.get("wrist");
        fingers = hardwareMap.servo.get("fingers");

//Initializing Servos
lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//Begin loop for program

        while (opModeIsActive()) {
//expand
//If-Then Logic for gamepad1
            //Forward & Backward
            if (gamepad1.left_stick_y != 0) {
                rf.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x);
                lf.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x);
                rb.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x);
                lb.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x);
            }
//If-Then logic for gamepad2
            if (gamepad2.a) {
                wrist.setPosition(0);
            }
            if (gamepad2.b) {
                fingers.setPosition(0.5);
            }
            if (gamepad2.x) {
                fingers.setPosition(0);
            }
            if (gamepad2.y) {
                wrist.setPosition(1);
            }

            stopRobot();
        }
    }

    private void stopRobot() {
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
    }
}