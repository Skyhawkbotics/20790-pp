package pedroPathing.autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.NanoTimer;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Config
@Autonomous(name = "LEFT AUTO", group = "AUTO")

public class left_auto extends OpMode {

    private Follower follower;

    // Timers
    private Timer opmodeTimer;
    private NanoTimer pathTimer; // Timers for progression of states, in nano seconds

    // Important Pose parts
    /*double pickup_x = 0.6; // Distance from the claw to the sample
    double turn_distance_x = 20; // Distance to start pickup
    double pickups_y = 41.00; // Observation Zone pickup distance*/

    //Initializing the states for the robot
    private int pathState, upArmState, outArmState, intakeState, outtakeState; // Different cases and states of the different parts of the robot

    // Poses, and pickUp Poses
    private Pose startPose = new Pose(10, 67.0, Math.toRadians(0)); //TODO
    private Pose bucketPose = new Pose(15, 129, Math.toRadians(135)); //TODO

    /// Paths, and path chains : pushFirst and pushSecond are called after hangFirst
    private Path preload, pickup1, basket1, pickup2, basket2, pickup3, basket3, park;
    //private PathChain ;//TODO: Trevor, idt we have any path chains, right?

    /// Motors
    private Servo servo_outtake_flip1, servo_outtake_flip2, servo_intake_wrist, servo_intake_rotate, sweeper, servo_intake, servo_outtake, servo_outtake_rotate;
    private DcMotorEx up1, up2, out;
    private TouchSensor up_zero, out_zero;
    //private TouchSensor up_zero;
    private Telemetry telemetryA;

    /// variables
    int up_basket_position = 0; //TODO: calibrate this value, slide position to



    public void buildPaths() { //TODO: Trevor needs to change these paths
        // preload
        preload = new Path(
                new BezierLine(
                        new Point(startPose),
                        new Point(bucketPose)
                )
        );
        preload.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135));
        pickup1 = new Path(
                // Line 2
                new BezierLine(
                        new Point(bucketPose),
                        new Point(20.000, 122.000, Point.CARTESIAN)
                )
        );
        pickup1.setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0));
        basket1 = new Path(
                // Line 3
                new BezierLine(
                        new Point(20.000, 122.000, Point.CARTESIAN),
                        new Point(bucketPose)
                )
        );
        basket1.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135));
        pickup2 = new Path(
                // Line 4
                new BezierLine(
                        new Point(bucketPose),
                        new Point(26.000, 131.500, Point.CARTESIAN)
                )
        );
        pickup2.setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0));
        basket2 = new Path(
                // Line 5
                new BezierLine(
                        new Point(26.000, 131.500, Point.CARTESIAN),
                        new Point(bucketPose)
                )
        );
        basket2.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135));
        pickup3 = new Path(
                // Line 6
                new BezierLine(
                        new Point(bucketPose),
                        new Point(28.000, 131.300, Point.CARTESIAN)
                )
        );
        pickup3.setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(35));
        basket3 = new Path(
                // Line 7
                new BezierLine(
                        new Point(28.000, 131.300, Point.CARTESIAN),
                        new Point(bucketPose)
                )
        );
        basket3.setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(135));
        park = new Path(
                // Line 8
                new BezierCurve(
                        new Point(bucketPose),
                        new Point(58.400, 131.000, Point.CARTESIAN),
                        new Point(59.000, 95.300, Point.CARTESIAN)
                )
        );
        park.setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270));

    }


    public void autonomousPathUpdate() {
        switch (pathState) {

        }
    }

    public void autonomousActionUpdate() { //TODO: Ruben... Set up variables for all necessary values so it's easy to calibrate

    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }



    @Override
    public void loop() {

        follower.update();

        autonomousPathUpdate();
        autonomousActionUpdate();


        // Feedback to Driver Hub, states and timers
        telemetryA.addData("path state", pathState);

        telemetryA.addData("pathtimer elapsed time", pathTimer.getElapsedTimeSeconds());



        // Poses, follower error
        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData("heading", follower.getPose().getHeading());
        telemetryA.addData("Follower busy", follower.isBusy());
        telemetryA.addData("headingPID error", follower.headingError);
        telemetryA.update();
    }



        // We init the timers, telemetry, motors , and follower
    @Override
    public void init() {

        // Timers init
        pathTimer = new NanoTimer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Follower, and it's constants init
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // Telemetry init
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

        //setup arms init
        up1 = hardwareMap.get(DcMotorEx.class, "up1"); //DONE: Is this still true?
        up1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up1.setDirection(DcMotorSimple.Direction.REVERSE);

        up2 = hardwareMap.get(DcMotorEx.class, "up2"); //DONE: Is this still true?
        up2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up2.setDirection(DcMotorSimple.Direction.REVERSE);

        out = hardwareMap.get(DcMotorEx.class, "out");
        int charles = out.getCurrentPosition(); //TODO: I need documentation for this please!
        out.setTargetPosition(charles);
        out.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        out.setPower(1);

        // Servos init
        servo_intake = hardwareMap.get(Servo.class, "intake");
        servo_intake_rotate = hardwareMap.get(Servo.class, "intakeRotate");
        servo_intake_wrist = hardwareMap.get(Servo.class, "intakeWrist");


        servo_outtake_flip1 = hardwareMap.get(Servo.class, "ofip1");
        servo_outtake_flip2 = hardwareMap.get(Servo.class, "ofip2");

        servo_outtake = hardwareMap.get(Servo.class, "outtake");
        servo_outtake_rotate = hardwareMap.get(Servo.class, "outtaker");

        //Sensor init
        up_zero = hardwareMap.get(TouchSensor.class, "up_zero");

    }

    /** This method is called continuously after Init while waiting for "play", it's not necessary but its useful to give more time to build paths **/
    @Override
    public void init_loop() {


        // After 4 Seconds, Robot Initialization is complete
        if (opmodeTimer.getElapsedTimeSeconds() > 4) {
            telemetryA.addData("ready to cook cook cook cook cook", "Finished");
        }
    }
    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        //buildPaths();
        opmodeTimer.resetTimer();
        //TODO: Change all of these values below
        setPathState(2);

    }



    @Override
    public void stop() {
    }
}