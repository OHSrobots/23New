package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class im_gonna_work2 {

    @Autonomous(name = "AAA Thxgiving")
    public static class TestRoadRunnerv2 extends LinearOpMode {

        //Declare Servos and Arm Motor; Note that drive train motors are already declared in MecanumDrive file
        private DcMotor arm;
        private CRServo wrist;
        private Servo fingers;

        @Override
        public void runOpMode() {

            //Map the servos and motor
            arm = hardwareMap.dcMotor.get("Arm");
            wrist = hardwareMap.crservo.get("wrist");
            fingers = hardwareMap.servo.get("fingers");

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            Pose2d startPose = new Pose2d(-36, -63, Math.toRadians(90));
            drive.setPoseEstimate(startPose);

            //Code Goblin ~ Mr. Surge: \*_*/

            //Movement from start to 5-pt post (Pole B3)
            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    //Follow Path
                    .splineTo(new Vector2d(-35.4, -38.5), Math.toRadians(90))
                    .splineTo(new Vector2d(-35.4, -10.3), Math.toRadians(45))
                    //Grip cone with fingers
                    .addDisplacementMarker(0.1, () -> {
                        fingers.setPosition(1);
                    })
                    //Move arm up && tilted-up wrist
                    .addDisplacementMarker(30, () -> {
                        arm.setPower(1);
                        wrist.setPower(-0.4);
                    })
                    .build();

            //Movement backwards from 5-pt pole to red line
            Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), true)
                    //Follow Path
                    .splineTo(new Vector2d(-35.9, -37), Math.toRadians(-90))
                    .build();

            //Movement from red line to cone stack
            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                    //Follow Path
                    .splineTo(new Vector2d(-36, -19), Math.toRadians(-90))
                    .splineTo(new Vector2d(-60, -18), Math.toRadians(-170))
                    //Lateral Wrist and Opened Fingers
                    .addDisplacementMarker(12, () -> {
                        arm.setPower(1);
                        fingers.setPosition(0.);
                        wrist.setPower(0.1);
                    })
                    .addDisplacementMarker(14, () -> {
                        arm.setPower(0);
                    })
                    .build();

            //Back to red line
            Trajectory traj4 = drive.trajectoryBuilder(traj3.end(), true)
                    //Follow Path
                    .splineTo(new Vector2d(-35.9, -37), Math.toRadians(-90))
                    .build();
            Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                    //Follow Path
                    .splineTo(new Vector2d(-35.4, -10.3), Math.toRadians(45))

                    .addDisplacementMarker(0.1, () -> {
                        fingers.setPosition(1);
                    })
                    //Move arm up && tilted-up wrist
                    .addDisplacementMarker(1, () -> {
                        arm.setPower(1);
                        wrist.setPower(-0.4);
                    })
                    .build();

            waitForStart();

            if (isStopRequested()) return;

            //Preloaded Cone
            drive.followTrajectory(traj1);
            wrist.setPower(0.1);
            sleep(250);
            fingers.setPosition(0.);
            sleep(150);
            arm.setPower(-1);
            sleep(150);
            wrist.setPower(-0.4);
            fingers.setPosition(1);

            //Cone #2
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);
            fingers.setPosition(1);
            arm.setPower(0);
            sleep(200);
            wrist.setPower(-0.6);
            drive.followTrajectory(traj4);
            drive.followTrajectory(traj5);
            wrist.setPower(0.1);
            sleep(250);
            fingers.setPosition(0.);
            sleep(150);
            arm.setPower(-1);
            sleep(150);
            wrist.setPower(-0.4);
            //fingers.setPosition(1);

            //Cone #3
            drive.followTrajectory(traj2);
            fingers.setPosition(1);
            drive.followTrajectory(traj3);
            fingers.setPosition(1);
            arm.setPower(0);
            sleep(300);
            wrist.setPower(-0.6);
            drive.followTrajectory(traj4);
            


        }
    }
}