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
@Autonomous(name = "RIGHT AUTO", group = "AUTO")
///* This is the Spark invitational auto
/// We won spark 2nd in placement, 1st pick alliance, and also 2nd inspire.
/// This  is a 4+0 autonomous and park for the into the deep season, using bezier curves and linear interpolation, along with a nano-second timer
public class left_auto extends OpMode {

    private Follower follower;

    // Timers
    private Timer opmodeTimer;
    private NanoTimer pathTimer; // Timers for progression of states, in nano seconds

    // Important Pose parts
    double pickup_x = 0.6; // Distance from the claw to the sample
    double turn_distance_x = 20; // Distance to start pickup
    double pickups_y = 41.00; // Observation Zone pickup distance
    private int pathState, armState, outclawState, outgrabState; // Different cases and states of the different parts of the robot

    // Poses, and pickUp Poses
    private Pose startPose = new Pose(10, 67.0, Math.toRadians(0)); //TODO
    private Pose bucketPose = new Pose(15, 129, Math.toRadians(135)); //TODO

    /// Paths, and path chains : pushFirst and pushSecond are called after hangFirst
    private Path preload, hang1, pickup1, pickup2, pickup3, hang2, hang3, park;
    private PathChain pushFirst, pushSecond;

    // Motors
    private DcMotorEx up, out; // Slide motors
    private Servo servo_outtake_wrist, servo_intake_wrist;
    private CRServo servo_outtake, servo_intake;
    private TouchSensor up_zero;
    private Telemetry telemetryA;

    // variables
    int up_hanging_position = 1778; //DONE: calibrate this value, viper slide position to
    int up_hanging_position_done = 1559; //TODO: calibrate this value, position of viper slide when releasing after speciman is on the bar.

    public void buildPaths() {
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
        hang1 = new Path (
                        // Line 3
                        new BezierLine(
                                new Point(20.000, 122.000, Point.CARTESIAN),
                                new Point(bucketPose)
                        )
                );
                hang1.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135));
        pickup2 = new Path (
                        // Line 4
                        new BezierLine(
                                new Point(bucketPose),
                                new Point(26.000, 131.500, Point.CARTESIAN)
                        )
                );
                pickup2.setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0));
        hang2 = new Path (
                        // Line 5
                        new BezierLine(
                                new Point(26.000, 131.500, Point.CARTESIAN),
                                new Point(bucketPose)
                        )
                );
        hang2.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135));
        pickup3 = new Path(
                        // Line 6
                        new BezierLine(
                                new Point(bucketPose),
                                new Point(28.000, 131.300, Point.CARTESIAN)
                        )
                );
        pickup3.setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(35));
        hang3 = new Path (
                        // Line 7
                        new BezierLine(
                                new Point(28.000, 131.300, Point.CARTESIAN),
                                new Point(bucketPose)
                        )
                );
        hang3.setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(135));
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
    public void autonomousActionUpdate() {
            switch (armState) {
                case -1: // default stop
                    up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    break;
                case 0: //going to bottom position, then closing the loop
                    up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    telemetry.addData("Lowered position", true);
                    if (!up_zero.isPressed()) {
                        up.setPower(-1);
                    } else if (up_zero.isPressed()) {
                        setArmState(-1);
                    }
                    break;
                case 1: //going to hanging position, using run to position
                    up.setTargetPosition(up_hanging_position);
                    up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    up.setPower(1);
                    if(up.getCurrentPosition() > 400) {
                        setoutClawState(1);
                    }
                    break;
                case 3:
                    up.setTargetPosition(up_hanging_position_done);
                    up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    up.setPower(0.6);
                    break;

            }
        switch (outclawState) {
            case -1: // Init Pos
                servo_outtake_wrist.setPosition(0);
                telemetry.addData("claw position 1 ", true);
                break;
            case 1: // Hang ready Pos
                servo_outtake_wrist.setPosition(0.58);
                telemetry.addData("claw position 2", true);
                break;
            case 2: // Hang done Pos
                servo_outtake_wrist.setPosition(0.27);
                break;
            case 3: // PickUp Pos
                servo_outtake_wrist.setPosition(0.55);
                break;

        }
        switch (outgrabState) {
            case -1: // Init Pos
                servo_outtake.setPower(0);
                break;
            case 1: //release
                servo_outtake.setPower(1);
                break;
            case 2: //grab
                servo_outtake.setPower(-1);
                break;
            case 4:
                if(servo_outtake_wrist.getPosition() <= 0.3) {
                    servo_outtake.setPower(1);
                } else {
                    servo_outtake.setPower(0);
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void setArmState(int aState) {
        armState = aState;
    }
    public void setoutGrabState(int gstate) {
        outgrabState = gstate;
    }
    public void setoutClawState(int cState) {
        outclawState = cState;
    }


    @Override
    public void loop() {

        follower.update();

        autonomousPathUpdate();
        autonomousActionUpdate();


        // Feedback to Driver Hub, states and timers
        telemetryA.addData("path state", pathState);
        telemetryA.addData("arm state", armState);
        telemetryA.addData("claw state", outclawState);
        telemetryA.addData("out grab state", outgrabState);
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
        up = hardwareMap.get(DcMotorEx.class, "up");
        up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up.setDirection(DcMotorSimple.Direction.REVERSE);

        out = hardwareMap.get(DcMotorEx.class, "out");
        int charles = out.getCurrentPosition();
        out.setTargetPosition(charles);
        out.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        out.setPower(1);

        // Servos init
        servo_outtake = hardwareMap.get(CRServo.class,"outtake");
        servo_intake = hardwareMap.get(CRServo.class, "intake");
        servo_intake_wrist = hardwareMap.get(Servo.class, "intakeWrist");
        servo_outtake_wrist = hardwareMap.get(Servo.class, "outtakeWrist");
        servo_outtake_wrist.setPosition(0);

        // Sensor init
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
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(2);
        setArmState(-1);
        setoutGrabState(0);
        setoutClawState(0);
    }
    @Override
    public void stop() {
    }
}