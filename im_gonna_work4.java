package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class im_gonna_work4 {

    @Autonomous(name = "Micah roadrunner thing-a-ma-gig", group =  "B")

    public static class TestRoadRunnerv2 extends LinearOpMode {

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
telemetry.addData("1", "");
telemetry.update();
                SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
                Pose2d startPose = new Pose2d(-36, -63, Math.toRadians(90));
                drive.setPoseEstimate(startPose);

                //Code Goblin ~ Mr. Surge: \*_*/
                //Movement from start to 5-pt post (Pole B3)
                /*Trajectory traj1 = drive.trajectoryBuilder(startPose)
                        //Follow Path
                        .strafeRight(12)

                        .addDisplacementMarker(.1, () -> {
                            arm.setPower(1);
                        })
                        .addDisplacementMarker((1.9*accelFactor)+0.1, () -> {
                            arm.setPower(.1);
                        })

                        .build();*/
            telemetry.addData("2", "");
            telemetry.update();
                //Movement backwards from 5-pt pole to red line
                Trajectory traj2 = drive.trajectoryBuilder(startPose, true)
                        //Follow Path
                        .forward(2)
                        .addDisplacementMarker(.1, () -> {
                            arm.setPower(1);
                        })
                        .addDisplacementMarker((1.3*accelFactor)+0.4, () -> {
                            arm.setPower(.1);
                        })
                        .build();
            telemetry.addData("2.5", "");
            telemetry.update();
                //Movement from red line to cone stack
                Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                        //Follow Path
                        .strafeLeft(26)
                        .addDisplacementMarker(.1, () -> {
                            arm.setPower(1);
                        })
                        .addDisplacementMarker(.3, () -> {
                            arm.setPower(.1);
                        })
                        //Lateral Wrist and Opened Fingers
                        .addDisplacementMarker(10.5, () -> {

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
                        .forward(49.5)
                        .addDisplacementMarker(12, () -> {
                            arm.setPower(-1);

                        })
                        .addDisplacementMarker((4*accelFactor)+15.3, () -> {
                            arm.setPower(.1);

                        })
                        .build();

            telemetry.addData("3.5", "");
            telemetry.update();
            TrajectorySequence ts = drive.trajectorySequenceBuilder(traj4.end())
                    .setTurnConstraint(5, 3)
                    .turn(Math.toRadians(-87)) // Turns 45 degrees counter-clockwise
                    .build();
            Trajectory traj5 = drive.trajectoryBuilder(ts.end())
                    .forward(43.5)
                    .build();
            Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                    .back(20.5)
                    .addDisplacementMarker(.1, () -> {
                        arm.setPower(1);
                    })
                    .addDisplacementMarker((2*accelFactor - 0.1), () -> {
                        arm.setPower(.1);
                    })
                    .build();
            TrajectorySequence ts2 = drive.trajectorySequenceBuilder(traj6.end())
                    .setTurnConstraint(5, 5)
                    .turn(Math.toRadians(-48)) // Turns 45 degrees counter-clockwise
                    .build();
            Trajectory traj7 = drive.trajectoryBuilder(ts2.end())
                    .forward(3.5)
                    .build();
            Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                    .back(3.5)
                    .build();
            TrajectorySequence ts3 = drive.trajectorySequenceBuilder(traj8.end())
                    .setTurnConstraint(5, 5)
                    .turn(Math.toRadians(47.8)) // Turns 45 degrees counter-clockwise
                    .build();
            Trajectory traj9 = drive.trajectoryBuilder(ts3.end())
                    .forward(21.5)
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
                    .back(22)
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

                //cone one
                drive.followTrajectory(traj2);

                sleep(100);
                drive.followTrajectory(traj3);
                //movin to cone 2
                drive.followTrajectory(traj4);
                wrist.setPower(-1);
                drive.followTrajectorySequence(ts);
                sleep(100);
                drive.followTrajectory(traj5);
                //grab cone 2
                wrist.setPower(0.1);
                sleep(100);
                fingers.setPosition(1);
                sleep(300);
                drive.followTrajectory(traj6);
                drive.followTrajectorySequence(ts2);
                drive.followTrajectory(traj7);
                fingers.setPosition(0);//deposited cone 2
                //sleep(200);
//go for cone 3
                drive.followTrajectory(traj8);
                drive.followTrajectorySequence(ts3);
                drive.followTrajectory(traj9);
                fingers.setPosition(1);
                sleep(200);
                wrist.setPower(-1);
                sleep(200);

                drive.followTrajectory(traj10);
                wrist.setPower(0);
                //repeat
                drive.followTrajectorySequence(ts2);
                drive.followTrajectory(traj7);
                fingers.setPosition(0);//deposit cone 3
                //sleep(200);

                drive.followTrajectory(traj8);
                drive.followTrajectorySequence(ts3);
                drive.followTrajectory(traj9);
                //move arm down more to get cone 4
                arm.setPower(-1);
                sleep(70);
                arm.setPower(0);
                fingers.setPosition(1);
                sleep(200);
                wrist.setPower(-1);
                sleep(200);

                drive.followTrajectory(traj10);
                wrist.setPower(0);
                arm.setPower(1);
                sleep(70);
                arm.setPower(0);
                //repeat again
                drive.followTrajectorySequence(ts2);
                drive.followTrajectory(traj7);
                fingers.setPosition(0);//deposit cone 4
                //sleep(200);

                drive.followTrajectory(traj8);
                drive.followTrajectorySequence(ts3);
                wrist.setPower(-1);
                sleep(20002);
                drive.followTrajectory(traj9);
                //go for cone 5
                //down more
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