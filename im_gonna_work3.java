package org.firstinspires.ftc.teamcode.util;
//go to line 166
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import static org.firstinspires.ftc.teamcode.util.SamplePipeline.TYPE.BLUESIDE;
import static org.firstinspires.ftc.teamcode.util.SamplePipeline.TYPE.GREENSIDE;
import static org.firstinspires.ftc.teamcode.util.SamplePipeline.TYPE.REDSIDE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;

public class im_gonna_work3 {

    @Autonomous(name = "Micah roadrunner thing-a-ma-jig", group =  "B")


    public static class TestRoadRunnerv2 extends LinearOpMode {
        //new stuff
        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;

        static final double FEET_PER_METER = 3.28084;

        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        // UNITS ARE METERS
        double tagsize = 0.166;

        int ID_TAG_1 = 1;
        int ID_TAG_2 = 2;
        int ID_TAG_3 = 3;

        AprilTagDetection tagOfInterest = null;
        //end new stuff
        //Declare Servos and Arm Motor; Note that drive train motors are already declared in MecanumDrive file
        private DcMotorEx arm;
        private CRServo wrist;
        private Servo fingers;
        double accelFactor = 2;
        @Override
        public void runOpMode() {
                //Map the servos and motor
                arm = hardwareMap.get(DcMotorEx.class, "Arm");
                wrist = hardwareMap.crservo.get("wrist");
                fingers = hardwareMap.servo.get("fingers");
                //new stuff
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
            camera.setPipeline(aprilTagDetectionPipeline);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode)
                {

                }
            });

            telemetry.setMsTransmissionInterval(50);
            //end new stuff
telemetry.addData("1", "");
telemetry.update();
                SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
                Pose2d startPose = new Pose2d(-36, -63, Math.toRadians(90));
                drive.setPoseEstimate(startPose);

                //Code Goblin ~ Mr. Surge: \*_*/

            telemetry.addData("2", "");
            telemetry.update();
                //Movement backwards from 5-pt pole to red line
                Trajectory traj2 = drive.trajectoryBuilder(startPose, true)
                        //Follow Path
                        .forward(3)
                        .addDisplacementMarker(.1, () -> {
                            arm.setPower(1);
                        })
                        .addDisplacementMarker((1.3*accelFactor)+0.35, () -> {
                            arm.setPower(.1);
                        })
                        .build();
            telemetry.addData("2.5", "");
            telemetry.update();
                //Movement from red line to cone stack
                Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                        //Follow Path
                        .strafeRight(26)
                        //Lateral Wrist and Opened Fingers
                        .addDisplacementMarker(6.8, () -> {

                            fingers.setPosition(0);
                            wrist.setPower(-1);
                        })
                        /*.addDisplacementMarker(14, () -> {
                            arm.setPower(0);
                        })*/
                        .build();
            telemetry.addData("3", "");
            telemetry.update();
                //Back to red line //FIX
                Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                        .forward(45.5)
                        .addDisplacementMarker(12, () -> {
                            arm.setPower(-1);

                        })
                        .addDisplacementMarker((4*accelFactor)+14.5, () -> {
                            arm.setPower(.1);

                        })
                        .build();

            telemetry.addData("3.5", "");
            telemetry.update();
            TrajectorySequence ts = drive.trajectorySequenceBuilder(traj4.end())
                    .setTurnConstraint(5, 3)
                    .turn(Math.toRadians(93)) // Turns 45 degrees counter-clockwise
                    .build();
            Trajectory traj5 = drive.trajectoryBuilder(ts.end())
                    .forward(43.5)
                    .build();
            Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                    .back(22.5)
                    .addDisplacementMarker(.1, () -> {
                        arm.setPower(1);
                    })
                    .addDisplacementMarker((2*accelFactor - 0.1), () -> {
                        arm.setPower(.1);
                    })
                    .build();
            TrajectorySequence ts2 = drive.trajectorySequenceBuilder(traj6.end())
                    .setTurnConstraint(5, 5)
                    .turn(Math.toRadians(49)) // Turns 45 degrees counter-clockwise
                    .build();
            Trajectory traj7 = drive.trajectoryBuilder(ts2.end())
                    .forward(3.5)
                    .build();
            Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                    .back(3.5)
                    .build();
            TrajectorySequence ts3 = drive.trajectorySequenceBuilder(traj8.end())
                    .setTurnConstraint(5, 5)
                    .turn(Math.toRadians(-48.8)) // Turns 45 degrees counter-clockwise
                    .build();
            Trajectory traj9 = drive.trajectoryBuilder(ts3.end())
                    .forward(23)
                    .addDisplacementMarker(0.1, () -> {
                        arm.setPower(-1);

                    })
                    .addDisplacementMarker((accelFactor)+2.5, () -> {
                        arm.setPower(.1);

                    })
                    .build();
            Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                    .back(22)
                    .addDisplacementMarker(0.1, () -> {
                        arm.setPower(1);

                    })
                    .addDisplacementMarker((accelFactor)+2, () -> {
                        arm.setPower(.1);

                    })
                    .build();
            Trajectory left2 = drive.trajectoryBuilder(traj10.end())
                    .back(1.5)
                    .build();
            Trajectory left1 = drive.trajectoryBuilder(traj10.end())
                    .forward(22)
                    .build();
            Trajectory left3 = drive.trajectoryBuilder(traj10.end())
                    .back(26)
                    .build();

            telemetry.addData("4", "");
            telemetry.update();
            fingers.setPosition(1);
            telemetry.addData("good to go", "");
            telemetry.update();
            waitForStart();

            //if (isStopRequested()) return;

            //Preloaded Cone

            wrist.setPower(0);
            sleep(200);
            while (opModeIsActive()){
                //new stuff
                int side = 1;
       ===\
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                if(currentDetections.size() != 0) {
                    boolean tagFound = false;

                    for (AprilTagDetection tag : currentDetections) {
                        if (tag.id == ID_TAG_1 || tag.id == ID_TAG_2 || tag.id == ID_TAG_3) {
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }
                    }
                }
                //end new stuff
                //go forward
                drive.followTrajectory(traj2);

                sleep(100);
                //go to side and drop off cone 1 on the way
                drive.followTrajectory(traj3);
                //go forward and move arm down a little to prepare to pick up the next cone
                drive.followTrajectory(traj4);
                //move wrist up to lessen the chances of it breaking
                wrist.setPower(-1);
                //turn then go forward to grab cone 2
                drive.followTrajectorySequence(ts);
                sleep(100);
                drive.followTrajectory(traj5);
                //grab cone 2
                wrist.setPower(0.1);
                sleep(100);
                fingers.setPosition(1);
                sleep(300);
                //go back and get into position to drop cone 2
                drive.followTrajectory(traj6);
                drive.followTrajectorySequence(ts2);
                drive.followTrajectory(traj7);
                fingers.setPosition(0);//dropped cone 2
                //get into position to pick up cone 3
                drive.followTrajectory(traj8);
                drive.followTrajectorySequence(ts3);
                drive.followTrajectory(traj9);
                //grab cone 3
                fingers.setPosition(1);
                sleep(200);
                wrist.setPower(-1);
                sleep(200);
                //get into position for cone 3
                drive.followTrajectory(traj10);
                wrist.setPower(0);

                drive.followTrajectorySequence(ts2);
                drive.followTrajectory(traj7);
                fingers.setPosition(0);//drop cone 3
                //go to get cone 4
                drive.followTrajectory(traj8);
                drive.followTrajectorySequence(ts3);
                drive.followTrajectory(traj9);
                //move arm down more to get cone 4
                arm.setPower(-1);
                sleep(70);
                arm.setPower(0);
                //grab cone 4
                fingers.setPosition(1);
                sleep(200);
                wrist.setPower(-1);
                sleep(200);
                //go to drop cone 4
                drive.followTrajectory(traj10);
                wrist.setPower(0);
                arm.setPower(1);
                sleep(70);
                arm.setPower(0);
                drive.followTrajectorySequence(ts2);
                drive.followTrajectory(traj7);
                fingers.setPosition(0);//deposit cone 4
                //get ready to park

                drive.followTrajectory(traj8);
                drive.followTrajectorySequence(ts3);
                //park
                wrist.setPower(-1);
                //new stuff
                if(tagOfInterest.id == ID_TAG_1) {
                    drive.followTrajectory(left1);
                }
                if(tagOfInterest.id == ID_TAG_2) {
                    drive.followTrajectory(left2);
                }
                if(tagOfInterest.id == ID_TAG_3) {
                    drive.followTrajectory(left3);
                }
                //end new stuff
                //wait till end of autonoumous
                sleep(20002);
                drive.followTrajectory(traj9);
                sleep(50000);


            }


        }
        void lift(int lengthUsingInches, int velocity){
            int ticksPerInch = 200;
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double calcPosition = lengthUsingInches * ticksPerInch;
            int setPosition = (int) Math.round(calcPosition);
            arm.setTargetPosition(setPosition);
            arm.setVelocity(velocity*ticksPerInch);
            while (opModeIsActive() && arm.isBusy()) {
                telemetry.addData("position", arm.getCurrentPosition());
                telemetry.addData("is at target", !arm.isBusy());
                telemetry.update();
            }
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        }
    }

}