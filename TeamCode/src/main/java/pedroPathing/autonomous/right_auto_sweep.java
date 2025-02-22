package pedroPathing.autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.pedropathing.util.NanoTimer;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of first_sweep servos autonomously.
 * It is able to detect the team element using a huskylens and then use that information to go to the correct spike mark and backdrop position.
 * There are examples of different ways to build paths.
 * A custom action system have been created that can be based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 9/8/2024
 */

@Config
@Autonomous(name = "RIGHT_AUTO_sweep", group = "AUTO")
// 18.5 inches away from observation zone
public class right_auto_sweep extends OpMode {

    /*
    Scrimage notes -
    Pushall curve 1 needs to be adjusted based on new start position
    Somehow initize the intake wrist / misumi slide so they do not come out at all during auto,
    minimize time and make code more effiecent
    optimize path a bit more, reducing hte start position and others.
    decrease time from pick up
    Run consisency tests
    maybe make the
     */
    // cool
    private Follower follower; // THe drivetrain with the calculations needed for path following
    private Timer actionTimer, opmodeTimer, outtimer; // Timers for progression of states

    private NanoTimer pathTimer;


    private int pathState, armState, outclawState, outgrabState, inclawState, ingrabState, sweeperState; // Different cases and states of the different parts of the robot
    /** Create and Define Poses + Paths
     * Poses are built with first_sweep constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Centerstage, this would be blue far side/red human player station.)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y. **/
    //Start Pose
    private Pose startPose = new Pose(10, 59.0, Math.toRadians(0)); //TODO

    private Pose pickupPose = new Pose( 6, 30, Math.toRadians(180));
    private Pose hangPose = new Pose(36.5, 72, Math.toRadians(0)); // TODO runs on

    private Pose firsthangPose = new Pose(36.0,70,0);

    private Pose secondhangPose = new Pose(36.0,68,0);

    private Pose thirdhangPose = new Pose(36.0, 66,0);

    private Pose pushstart = new  Pose(60,30,0);

    private Pose firstpushPose = new Pose(20,29, Math.toRadians(0));

    private Pose pushstart2 = new Pose(60,22,0);

    private Pose endPush = new Pose(20,18, Math.toRadians(0));

    private Pose readyPose = new Pose(20,37, Math.toRadians(180));

    private Pose parkPose = new Pose(10,24,0);

    private Pose readyPose1 = new Pose(20,18, Math.toRadians(180));

    private Pose pickupPose1 = new Pose(6 ,18,Math.toRadians(180));



    // Paths
    private PathChain preload_hang, back1, back2, back3;

    private Path to_sweep, first_sweep, second_sweep_ready, second_sweep, third_sweep_ready, third_sweep, pickup1, hang1, hang2, hang3, hang4, fourteen, fifteen, sixteen, seventeen, eighteen;

    // Motors
    private DcMotorEx up, out;
    private Servo servo_outtake_wrist, servo_intake_wrist, servo_intake_rotate, sweeper;
    private CRServo servo_outtake, servo_intake;
    private TouchSensor up_zero, out_zero;
    private Telemetry telemetryA;
    double intake_wrist_pos_transfer = 0;
    double outtake_wrist_pos_transfer = 0;
    int up_hanging_position = 1775; //DONE: calibrate this value, viper slide position to
    int up_hanging_position_done = 1270; //TODO: calibrate this value, position of viper slide when releasing after speciman is on the bar.

    int preload_hang_pos = 2160;
    // 1543
    //0.29

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        preload_hang = follower.pathBuilder()
                .addPath(
                new BezierLine(
                        new Point(10.000, 59.000, Point.CARTESIAN),
                        new Point(35, 59.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(0)
                .setZeroPowerAccelerationMultiplier(3)
                .build();
        to_sweep = new Path(
                new BezierCurve(
                        new Point(34.000, 59.000, Point.CARTESIAN),
                        new Point(31.683, 53.644, Point.CARTESIAN),
                        new Point(36, 30.000, Point.CARTESIAN) //
                )
        );
        to_sweep.setLinearHeadingInterpolation(0, Math.toRadians(300));
        first_sweep = new Path(
                new BezierLine(
                        new Point(37, 30.000, Point.CARTESIAN), //
                        new Point(15.000, 32.000, Point.CARTESIAN)
                )
        );
        first_sweep.setLinearHeadingInterpolation(Math.toRadians(300), Math.toRadians(240), 0.6);
        second_sweep_ready = new Path(
                new BezierLine(
                        new Point(15.000, 32, Point.CARTESIAN),
                        new Point(37, 21.000, Point.CARTESIAN)
                )
        );
        second_sweep_ready.setLinearHeadingInterpolation(Math.toRadians(240), Math.toRadians(290), 0.9);
        second_sweep = new Path(
                new BezierLine(
                        new Point(37, 21.000, Point.CARTESIAN),
                        new Point(17, 22.000, Point.CARTESIAN)
                )
        );
        second_sweep.setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(240), 0.6);
        third_sweep_ready = new Path(
                new BezierLine(
                        new Point(17, 22, Point.CARTESIAN),
                        new Point(36, 11, Point.CARTESIAN)
                )
        );
        third_sweep_ready.setLinearHeadingInterpolation(Math.toRadians(240), Math.toRadians(290), 0.6);
        third_sweep = new Path(
                new BezierLine(
                        new Point(36, 11, Point.CARTESIAN),
                        new Point(15, 14, Point.CARTESIAN)
                )
        );
        third_sweep.setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(180));
        pickup1 = new Path(
                new BezierLine(
                        new Point(15, 10, Point.CARTESIAN),
                        new Point(3, 10, Point.CARTESIAN)
                )
        );
        pickup1.setConstantHeadingInterpolation(Math.toRadians(180));
        hang1 = new Path(
                new BezierCurve(
                        new Point(3, 10, Point.CARTESIAN),
                        new Point(36.500, 63, Point.CARTESIAN)
                )
        );
        hang1.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0));
        back1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(36.5, 63, Point.CARTESIAN),
                                new Point(20, 35, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(0, Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(4.5)
                .addPath(
                        new BezierLine(
                                new Point(20, 35, Point.CARTESIAN),
                                new Point(3, 35, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        hang2 = new Path(
                new BezierLine(
                        new Point(3, 35, Point.CARTESIAN),
                        new Point(36.5, 61, Point.CARTESIAN)
                )
        );
        hang2.setLinearHeadingInterpolation(Math.toRadians(180), 0);
        back2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(36.5, 61, Point.CARTESIAN),
                                new Point(20, 35, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(0, Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Point(20, 35, Point.CARTESIAN),
                                new Point(3, 35, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        hang3 = new Path(
                new BezierCurve(
                        new Point(3.000, 35.000, Point.CARTESIAN),
                        new Point(36.000, 59, Point.CARTESIAN)
                )
        );
        hang3.setLinearHeadingInterpolation(Math.toRadians(180), 0);
        back3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(36, 59, Point.CARTESIAN),
                                new Point(20, 35, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(0, Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Point(20, 35, Point.CARTESIAN),
                                new Point(3, 35, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        hang4 = new Path(
                new BezierCurve(
                        new Point(3, 35, Point.CARTESIAN),
                        new Point(36.5, 57, Point.CARTESIAN)
                )
        );
        hang4.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0));

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 1:
                setArmState(1); // init hang
                follower.followPath(preload_hang, true);
                setPathState(2);
                break;
            case 2:
                if (!follower.isBusy()) { // wait for stop and also hang
                    setPathState(3);
                }
                break;
            case 3:
                if(pathTimer.getElapsedTime() > (0.2*(Math.pow(10,9)))) { // release
                    follower.followPath(to_sweep);
                    setSweeperState(3);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    setArmState(0); // lower arm
                    setoutGrabState(-1);
                    setSweeperState(1); // sweeper push pos
                    setPathState(50);
                }
                break;
            case 50:
                if(pathTimer.getElapsedTime() > (0.2*(Math.pow(10,9)))) { // release
                    follower.followPath(first_sweep);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    setSweeperState(2);
                    follower.followPath(second_sweep_ready);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    setSweeperState(1);
                    setPathState(70);
                }
                break;
            case 70:
                if(pathTimer.getElapsedTime() > (0.2*(Math.pow(10,9)))) { // release
                    follower.followPath(second_sweep);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    setSweeperState(2);
                    follower.followPath(third_sweep_ready);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    setSweeperState(1);
                    setPathState(9);
                }
                break;
            case 9:
                if(pathTimer.getElapsedTime() > (0.2*(Math.pow(10,9)))) { // release
                    setSweeperState(4);
                    follower.followPath(third_sweep);
                    setoutClawState(3); // set claw
                    setoutGrabState(2); // grab
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(pickup1);
                    setPathState(110);
                }
                break;
            case 110:
                if(pathTimer.getElapsedTime() > (0.2*(Math.pow(10,9)))) { // pickup time :D
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(hang1);
                    setoutGrabState(4); // unstable
                    setoutClawState(1);
                    setArmState(2);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) { // hang
                    setArmState(3);
                    setoutClawState(2);
                    setPathState(130);
                }
                break;
            case 130:
                if(pathTimer.getElapsedTime() > (0.15*(Math.pow(10,9)))) { // release
                    follower.followPath(back1);
                    setPathState(13);
                }
                break;
            case 13:
                if(pathTimer.getElapsedTime() > (0.35*(Math.pow(10,9)))) { // wait for lower
                    setArmState(0);
                    setoutClawState(3);
                    setoutGrabState(2);
                    setPathState(131);
                }
                break;
            case 131:
                if (!follower.isBusy()) { // pickup
                    setPathState(140);
                }
                break;
            case 140:
                if(pathTimer.getElapsedTime() > (0.2*(Math.pow(10,9)))) { // pickup time :D
                    setPathState(141);
                }
            case 141:
                follower.followPath(hang2);
                setArmState(2);
                setoutGrabState(4);
                setPathState(14);
                break;
            case 14: // hang
                if (!follower.isBusy()) {
                    setArmState(3);
                    setoutClawState(2);
                    setPathState(150);
                }
                break;
            case 150:
                if(pathTimer.getElapsedTime() > (0.15*(Math.pow(10,9)))) { // release
                    follower.followPath(back2);
                    setPathState(151);
                }
                break;
            case 151:
            if(pathTimer.getElapsedTime() > (0.35*(Math.pow(10,9)))) { // wait for lower and start intake
                setArmState(0);
                setoutClawState(3);
                setoutGrabState(2);
                setPathState(15);
            }
            break;
            case 15:
                if (!follower.isBusy()) {
                    setPathState(160);
                }
                break;
            case 160:
                if(pathTimer.getElapsedTime() > (0.2*(Math.pow(10,9)))) { // pickup time :D
                    setPathState(161);
                }
                break;
            case 161:
// raise and go
                    setArmState(2);
                    setoutGrabState(4);
                    setoutClawState(1);
                    follower.followPath(hang3);
                    setPathState(16);
                break;
            case 16:
                if (!follower.isBusy()) {
                    setArmState(3);
                    setoutClawState(2);
                    setPathState(170);
                }
                break;
            case 170:
                if(pathTimer.getElapsedTime() > (0.15*(Math.pow(10,9)))) { // release
                    follower.followPath(back3);
                    setPathState(181);
                }
                break;
            case 181:
                if(pathTimer.getElapsedTime() > (0.35*(Math.pow(10,9)))) { // wait for lower and start intake
                    setArmState(0);
                    setoutClawState(3);
                    setoutGrabState(2);
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    setPathState(20);
                }
                break;
            case 20:
                if(pathTimer.getElapsedTime() > (0.2*(Math.pow(10,9)))) { // intake time
                    setArmState(2);
                    setoutGrabState(4);
                    setoutClawState(1);
                    follower.followPath(hang4);
                    setPathState(21);
                }
                break;
            case 21:
                if (!follower.isBusy()) {
                    setArmState(3);
                    setoutClawState(2);
                }
                break;

        }
    }

    /** This switch is called continuously and runs the necessary actions, when finished, it will set the state to -1.
     * (Therefore, it will not run the action continuously) **/
    public void autonomousActionUpdate() {
        switch (armState) {
            case -1:
                up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//most of the code stolen from opmode_main
                break;
            case 0: //going to bottom position
                up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Lowered position", true);
                if (!up_zero.isPressed()) {
                    up.setPower(-1);
                } else if (up_zero.isPressed()) {
                    setArmState(-1);
                }
                break;
            case 1: // preload
                up.setTargetPosition(preload_hang_pos);
                up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                up.setPower(1);
                if(up.getCurrentPosition() > 350) {
                    setoutClawState(4);
                }
                if(up.getCurrentPosition() >= 2160) {
                    setoutGrabState(1);
                }
                break;

            case 2: //going to hanging position
                up.setTargetPosition(up_hanging_position);
                up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                up.setPower(1);
                break;
            case 3:
                up.setTargetPosition(up_hanging_position_done);
                up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                up.setPower(1);
                break;

        }
        switch (outclawState) {
            case -1:
                servo_outtake_wrist.setPosition(0);
                break;
            case 1:
                servo_outtake_wrist.setPosition(0.58);
                break;
            case 2: // Hang done Pos
                servo_outtake_wrist.setPosition(0.25);
                servo_intake_wrist.setPosition(0);
                break;
            case 3:
                servo_outtake_wrist.setPosition(0.47);
                break;
            case 4:
                servo_outtake_wrist.setPosition(1);
                break;

        }
        switch (outgrabState) {
            case -1:
                servo_outtake.setPower(0);
                break;
            case 1: //release
                servo_outtake.setPower(1);
                break;
            case 2: //grab
                servo_outtake.setPower(-1);
                break;
            case 3: // hang release?
                if (up.getCurrentPosition() < 1600) {
                    servo_outtake.setPower(1);
                }
            case 4:
                if(servo_outtake_wrist.getPosition() <= 0.3) {
                    servo_outtake.setPower(1);
                } else {
                    servo_outtake.setPower(0);
                }
                break;
        }
        switch (inclawState) {
            case 0:
                servo_intake_wrist.setPosition(0);
                break;
            case 1:
                servo_intake_wrist.setPosition(0.8);
                break;

        }
        switch (ingrabState) {
            case 0:
                servo_intake.setPower(0);
                break;
            case 1: // Release?
                servo_intake.setPower(1);
                break;
            case 2:
                servo_intake.setPower(-1);
                break;

        }
        switch (sweeperState) {
            case -1: // init
                sweeper.setPosition(0.8);
                break;
            case 1: //release
                sweeper.setPosition(0.14);
                break;
            case 2:
                sweeper.setPosition(0.20);
                break;
            case 3: // parked
                sweeper.setPosition(0.45);
                break;
            case 4:
                if(follower.getPose().getHeading() < Math.toRadians(180)) {
                    sweeper.setPosition(0.4);
                }
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
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
    public void setinclawState(int icstate) {
        inclawState = icstate;
    }
    public void setIngrabState(int icstate) {
        ingrabState = icstate;
    }
    public void setoutClawState(int cState) {
        outclawState = cState;
    }
    public void setSweeperState(int mstate) {
        sweeperState = mstate;
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the actions and movement of the robot
        follower.update();

        autonomousPathUpdate();
        autonomousActionUpdate();


        // Feedback to Driver Hub
        telemetryA.addData("path state", pathState);
        telemetryA.addData("arm state", armState);
        telemetryA.addData("claw state", outclawState);
        telemetryA.addData("out grab state", outgrabState);

        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData("heading", follower.getPose().getHeading());
        //telemetryA.addData("armPOS", up.getCurrentPosition());
        //telemetryA.addData("out servo", servo_outtake_wrist.getPosition());
        telemetryA.addData("pathtimer elapsed time", pathTimer.getElapsedTimeSeconds());
        //telemetryA.addData("armmode", up.getMode());
        telemetryA.addData("Follower busy", follower.isBusy());

        telemetryA.addData("headingPID error", follower.headingError);
        telemetryA.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new NanoTimer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);


        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

        //setup arm variable

        up = hardwareMap.get(DcMotorEx.class, "up");
        up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up.setDirection(DcMotorSimple.Direction.REVERSE);

        //example position setup
        out = hardwareMap.get(DcMotorEx.class, "out");
        int charles = out.getCurrentPosition();
        out.setTargetPosition(charles);
        out.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        out.setPower(1);

        servo_outtake = hardwareMap.get(CRServo.class,"outtake");


        servo_intake = hardwareMap.get(CRServo.class, "intake");

        sweeper = hardwareMap.get(Servo.class, "sweeper");
        sweeper.setPosition(0.8);


        servo_intake_wrist = hardwareMap.get(Servo.class, "intakeWrist");

        servo_outtake_wrist = hardwareMap.get(Servo.class, "outtakeWrist");
        servo_outtake_wrist.setPosition(0);


        up_zero = hardwareMap.get(TouchSensor.class, "up_zero");


    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {


        // After 4 Seconds, Robot Initialization is complete
        if (opmodeTimer.getElapsedTimeSeconds() > 4) {
            telemetryA.addData("Init", "Finished");
        }
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        //setBackdropGoalPose();
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(1);
        setArmState(-1); //starting ArmState
        setoutGrabState(0);
        setinclawState(0);
        setIngrabState(0);
        setoutClawState(0);


    }
    // run this



    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
