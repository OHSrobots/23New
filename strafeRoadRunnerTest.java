
package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

public class strafeRoadRunnerTest {

    @Autonomous(name = "strafeRoadRunnerTest")
    public static class TestRoadRunnerv2 extends LinearOpMode {
        @Override
        public void runOpMode() {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            Pose2d startPose = new Pose2d(-36, -65.5, Math.toRadians(90));
            drive.setPoseEstimate(startPose);
            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    .strafeRight(20)
                    .build();
            Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), false)
                    .splineTo(new Vector2d(-34, -24), Math.toRadians(0))
                    .build();
            /*Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), true)
                    .splineTo(new Vector2d(-41, -24), Math.toRadians(0))
                    .build();
            /*Trajectory traj4 = drive.trajectoryBuilder(traj3.end(), false)
                    .splineTo(new Vector2d(-39.5, -24), Math.toRadians(37))
                    .build();
            Trajectory traj4 = drive.trajectoryBuilder(traj3.end(), true)
                    .splineTo*/


            /*
            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                    .splineTo(new Vector2d(-60, -36), Math.toRadians(-180))
                    //.splineToLinearHeading(new Pose2d(24,-12, Math.toRadians(90)), Math.toRadians(90))
                    .build();


    */
    /*
            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(90)), Math.toRadians(45))
                  //  .splineTo(new Vector2d(-60, -36), Math.toRadians(0))
                    .build();
            Trajectory traj2 = drive.trajectoryBuilder(new Pose2d())
                    .back(17)
                    .build();
            Trajectory traj3 = drive.trajectoryBuilder(new Pose2d())
                    .forward(17)
                    .build();
            Trajectory traj4 = drive.trajectoryBuilder(traj1.end())
                    .splineToSplineHeading(new Pose2d(-44, -20, Math.toRadians(45)), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(-26, -12, Math.toRadians(0)), Math.toRadians(45))
                    .build();
            Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                    .splineToLinearHeading(new Pose2d(-60, -36, Math.toRadians(45)), Math.toRadians(0))
                    .build();
    */
    /*
            drive.setPoseEstimate(startPose);
            Trajectory traj2 = drive.trajectoryBuilder(startPose)
                    //.splineToSplineHeading(new Pose2d(-24, -48, Math.toRadians(45)), Math.toRadians(0))

                    .splineTo(new Vector2d(-24, -48), Math.toRadians(45))
                    .build();
            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                    //.splineToSplineHeading(new Pose2d(-48, -48, Math.toRadians(45)), Math.toRadians(45))
                    .splineTo(new Vector2d(-48, -48), Math.toRadians(45))
                    .build();
            Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                    .back(17)
                    .build();
            Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                    .forward(17)
                    .build();
            Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                    //.splineToSplineHeading(new Pose2d(-60, -36, Math.toRadians(45)), Math.toRadians(-180))
                    .splineTo(new Vector2d(-60, -36), Math.toRadians(-180))
                    .build();*/
            /*Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(-0, -48), Math.toRadians(135))
                    .build();
            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                    .back(12)
                    .build();
            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                    .splineTo(new Vector2d(-48, -48), Math.toRadians(45))
                    .build();
            Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                    .back(17)
                    .build();
            Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                    .forward(17)
                    .build();
            Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                    .splineTo(new Vector2d(-60, -36), Math.toRadians(-180))
                    .build();
            Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                    .back(12)
                    .build();
            Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                    .splineTo(new Vector2d(-24, -67), Math.toRadians(0))
                    .splineTo(new Vector2d(36, -67), Math.toRadians(0))
                    .splineTo(new Vector2d(48, -60), Math.toRadians(30))
                    .build();
            Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                    .splineTo(new Vector2d(48, -67), Math.toRadians(30))
                    .build();
            Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                    .back(36)
                    .build();
    */
            waitForStart();

            if (isStopRequested()) return;
            drive.followTrajectory(traj1);

            //drive.followTrajectory(traj2);
            //drive.followTrajectory(traj3);
            //drive.followTrajectory(traj4);
            //drive.followTrajectory(traj5);


        }
    }

    /**
     * This OpMode illustrates using a webcam to locate and drive towards ANY Vuforia target.
     * The code assumes a basic two-wheel Robot Configuration with motors named left_drive and right_drive.
     * The motor directions must be set so a positive drive goes forward and a positive turn rotates to the right.
     * <p>
     * Under manual control, the left stick will move forward/back, and the right stick will turn left/right.
     * This is called POV Joystick mode, different than Tank Drive (where each joystick controls a wheel).
     * Manually drive the robot until it displays Target data on the Driver Station.
     * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
     * Release the Left Bumper to return to manual driving mode.
     * <p>
     * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
     * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN and TURN_GAIN constants.
     * <p>
     * For more Vuforia details, or to adapt this OpMode for a phone camera, view the
     * ConceptVuforiaFieldNavigation and ConceptVuforiaFieldNavigationWebcam samples.
     * <p>
     * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
     * <p>
     * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
     * is explained below.
     */

    @TeleOp(name = "Drive To Target50", group = "Concept")
    public static class ConceptVuforiaDriveToTargetWebcam extends LinearOpMode {
        // Adjust these numbers to suit your robot.
        final double DESIRED_DISTANCE = 0.2; //  this is how close the camera should get to the target (inches)
        //  The GAIN constants set the relationship between the measured position error,
        //  and how much power is applied to the drive motors.  Drive = Error * Gain
        //  Make these values smaller for smoother control.
        final double SPEED_GAIN = 0.02;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double TURN_GAIN = 0.02;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        final double MM_PER_INCH = 25.40;   //  Metric conversion

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code on the next line, between the double quotes.
         */
        private static final String VUFORIA_KEY = "ARxaOAX/////AAABmR91q9ci+kNYqGb/NElhuhBQa5klidYZ5jKk5hFYJ6qAQOtCGKSEZXn1qYawipXKEEpJh+vP3GNnOUvabO2blz4vkymDnu8LUocLc6/rMpQdLwBt80JVdgWWkd/4j1DmwDdRRP4f/jP78furjgexjT7HgmC37xLP+msr78zAeWwkrsT2X1yjnL6nyiGcRKlBw6+EcUIcZYiiuXwbILds8rl4Fu7AuecLaygDft6XIUFg/qQm51UF45l5pYT8AoNTUhP9GTksKkmHgde7iGlo3CfIYu9QanjPHreT/+JZLJWG22jWC7Nnzch/1HC6s3s2jzkrFV6sRVA4lL9COLIonjRBYPhbxCF06c5fUMy9sj/e";

        VuforiaLocalizer vuforia = null;
        OpenGLMatrix targetPose = null;
        String targetName = "";

        private DcMotorEx leftFront = null;
        private DcMotorEx rightFront = null;
        private DcMotorEx leftBack = null;
        private DcMotorEx rightBack = null;

        @Override
        public void runOpMode() {
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             * To get an on-phone camera preview, use the code below.
             * If no camera preview is desired, use the parameter-less constructor instead (commented out below).
             */
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;

            // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
            parameters.useExtendedTracking = false;

            // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 2");
            this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Load the trackable objects from the Assets file, and give them meaningful names
            VuforiaTrackables targetsFreightFrenzy = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");
            targetsFreightFrenzy.get(0).setName("Blue Storage");
            targetsFreightFrenzy.get(1).setName("Blue Alliance Wall");
            targetsFreightFrenzy.get(2).setName("Red Storage");
            targetsFreightFrenzy.get(3).setName("Red Alliance Wall");

            // Start tracking targets in the background
            targetsFreightFrenzy.activate();

            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).
            rightFront = hardwareMap.get(DcMotorEx.class, "rightfront");
            leftFront = hardwareMap.get(DcMotorEx.class, "leftfront");
            rightBack = hardwareMap.get(DcMotorEx.class, "rightback");
            leftBack = hardwareMap.get(DcMotorEx.class, "leftback");

            // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
            // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
            rightFront.setDirection(DcMotorEx.Direction.REVERSE);
            leftFront.setDirection(DcMotorEx.Direction.FORWARD);
            rightBack.setDirection(DcMotorEx.Direction.REVERSE);
            leftBack.setDirection(DcMotorEx.Direction.FORWARD);

            telemetry.addData(">", "Press Play to start");
            telemetry.update();

            waitForStart();

            boolean targetFound = false;    // Set to true when a target is detected by Vuforia
            double targetRange = 0;        // Distance from camera to target in Inches
            double targetBearing = 0;        // Robot Heading, relative to target.  Positive degrees means target is to the right.
            double drive = 0;        // Desired forward power (-1 to +1)
            double turn = 0;        // Desired turning power (-1 to +1)

            while (opModeIsActive()) {
                // Look for first visible target, and save its pose.
                targetFound = false;
                for (VuforiaTrackable trackable : targetsFreightFrenzy) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        targetPose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getVuforiaCameraFromTarget();

                        // if we have a target, process the "pose" to determine the position of the target relative to the robot.
                        if (targetPose != null) {
                            targetFound = true;
                            targetName = trackable.getName();
                            VectorF trans = targetPose.getTranslation();

                            // Extract the X & Y components of the offset of the target relative to the robot
                            double targetX = trans.get(0) / MM_PER_INCH; // Image X axis
                            double targetY = trans.get(2) / MM_PER_INCH; // Image Z axis

                            // target range is based on distance from robot position to origin (right triangle).
                            targetRange = Math.hypot(targetX, targetY);

                            // target bearing is based on angle formed between the X axis and the target range line
                            targetBearing = Math.toDegrees(Math.asin(targetX / targetRange));

                            break;  // jump out of target tracking loop if we find a target.
                        }
                    }
                }

                // Tell the driver what we see, and what to do.
                if (targetFound) {
                    telemetry.addData(">", "HOLD Left-Bumper to Drive to Target\n");
                    telemetry.addData("Target", " %s", targetName);
                    telemetry.addData("Range", "%5.1f inches", targetRange);
                    telemetry.addData("Bearing", "%3.0f degrees", targetBearing);
                } else {
                    telemetry.addData(">", "Drive using joystick to find target\n");
                }

                // Drive to target Automatically if Left Bumper is being pressed, AND we have found a target.
                if (gamepad1.left_bumper && targetFound) {

                    // Determine heading and range error so we can use them to control the robot automatically.
                    double rangeError = (targetRange - DESIRED_DISTANCE);
                    double headingError = targetBearing;

                    // Use the speed and turn "gains" to calculate how we want the robot to move.
                    drive = rangeError * SPEED_GAIN;
                    turn = headingError * TURN_GAIN;

                    telemetry.addData("Auto", "Drive %5.2f, Turn %5.2f", drive, turn);
                } else {

                    // drive using manual POV Joystick mode.
                    drive = -gamepad1.left_stick_y / 2.0;  // Reduce drive rate to 50%.
                    turn = gamepad1.right_stick_x / 4.0;  // Reduce turn rate to 25%.
                    telemetry.addData("Manual", "Drive %5.2f, Turn %5.2f", drive, turn);
                }
                telemetry.update();

                // Calculate left and right wheel powers and send to them to the motors.
                double leftPower = Range.clip(drive - turn, -1.0, 1.0);
                double rightPower = Range.clip(drive + turn, -1.0, 1.0);
                leftFront.setPower(leftPower);
                rightFront.setPower(rightPower);
                leftBack.setPower(leftPower);
                rightBack.setPower(rightPower);

                sleep(10);
            }
        }
    }
}
