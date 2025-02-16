package pedroPathing.autonomous;


import android.os.storage.StorageManager;

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

import java.net.BindException;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of three servos autonomously.
 * It is able to detect the team element using a huskylens and then use that information to go to the correct spike mark and backdrop position.
 * There are examples of different ways to build paths.
 * A custom action system have been created that can be based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 9/8/2024
 */

@Config
@Autonomous(name = "RIGHT_AUTO", group = "AUTO")
// 18.5 inches away from observation zone
public class right_auto extends OpMode {

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


    private int pathState, armState, outclawState, outgrabState, inclawState, ingrabState; // Different cases and states of the different parts of the robot
    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Centerstage, this would be blue far side/red human player station.)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y. **/
    //Start Pose
    private Pose startPose = new Pose(10, 67.0, Math.toRadians(0)); //TODO

    private Pose pickupPose = new Pose( 3, 36, Math.toRadians(180));
    private Pose hangPose = new Pose(36.5, 72, Math.toRadians(0)); // TODO runs on

    private Pose firsthangPose = new Pose(36.0,70,0);

    private Pose secondhangPose = new Pose(36.0,68,0);

    private Pose thirdhangPose = new Pose(36.0, 66,0);

    private Pose pushstart = new  Pose(60,30,0);

    private Pose firstpushPose = new Pose(20,29, Math.toRadians(0));

    private Pose pushstart2 = new Pose(60,22,0);

    private Pose endPush = new Pose(20,18, Math.toRadians(0));

    private Pose readyPose = new Pose(20,30, Math.toRadians(180));

    private Pose parkPose = new Pose(10,24,0);

    private Pose readyPose1 = new Pose(20,18, Math.toRadians(180));

    private Pose pickupPose1 = new Pose(3,18,Math.toRadians(180));



    // Paths
    private PathChain hang1;

    private Path hang_first, park;
    private Path pushAll1, pushAll2, pushAll3, pushAll4, pushAll5, pushAll6, pushAll7, pushAll8, pickup1;

    private Path ready_pickup, pickup, first_hang, first_hang_back, second_hang, second_hang_back, third_hang, third_hang_back, fourth_hang, fourth_hang_back;

    // Motors
    private DcMotorEx up, out;
    private Servo servo_outtake_wrist, servo_intake_wrist, servo_intake_rotate;
    private CRServo servo_outtake, servo_intake;
    private TouchSensor up_zero, out_zero;
    private Telemetry telemetryA;
    double intake_wrist_pos_transfer = 0;
    double outtake_wrist_pos_transfer = 0;
    int up_hanging_position = 1755; //DONE: calibrate this value, viper slide position to
    int up_hanging_position_done = 1250; //TODO: calibrate this value, position of viper slide when releasing after speciman is on the bar.
    // 1543
    //0.29

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        hang_first = new Path(
                new BezierLine(
                        new Point(startPose),
                        new Point(hangPose)
                )
        );
        hang_first.setConstantHeadingInterpolation(hangPose.getHeading());
        hang_first.setZeroPowerAccelerationMultiplier(1.5);

        pushAll1 = new Path(
                new BezierCurve(
                        new Point(hangPose),
                        new Point(22.1, 20.0, Point.CARTESIAN),
                        new Point(56.51, 47.2, Point.CARTESIAN),
                        new Point(pushstart)
                )
        );
        pushAll1.setConstantHeadingInterpolation(0);
        pushAll1.setZeroPowerAccelerationMultiplier(1.25);
        pushAll3 = new Path( // goes back
                new BezierLine(
                        new Point(pushstart),
                        new Point(firstpushPose)
                )
        );
        pushAll3.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        pushAll3.setZeroPowerAccelerationMultiplier(1.5);
        pushAll4 = new Path (
                new BezierCurve(
                        new Point(24.000, 29.000, Point.CARTESIAN),
                        new Point(63.174, 31.123, Point.CARTESIAN),
                        new Point(pushstart2)
                )
        );
        pushAll4.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)); // curve toget in front of second sample
        pushAll5 = new Path(
                new BezierLine(
                        new Point(pushstart2),
                        new Point(endPush)
                )
        );
        pushAll5.setConstantHeadingInterpolation(endPush.getHeading());
        pushAll5.setZeroPowerAccelerationMultiplier(1.5);
        ready_pickup = new Path(
                // Line 1
                new BezierLine(
                        new Point(endPush),
                        new Point(readyPose)
                )
        );
        ready_pickup.setLinearHeadingInterpolation(endPush.getHeading(), readyPose.getHeading());
        pickup = new Path(
                // Line 2
                new BezierLine(
                        new Point(readyPose),
                        new Point(pickupPose)
                )
        );
        pickup.setConstantHeadingInterpolation(pickupPose.getHeading());
        pickup.setZeroPowerAccelerationMultiplier(1);
        first_hang = new Path(
                // Line 3
                new BezierCurve(
                        new Point(pickupPose),
                        new Point(15.5, 63, Point.CARTESIAN),
                        new Point(firsthangPose)
                )
        );
        first_hang.setLinearHeadingInterpolation(pickupPose.getHeading(), Math.toRadians(0));
        first_hang.setZeroPowerAccelerationMultiplier(1.25);
        first_hang_back = new Path(
                // Line 4
                new BezierCurve(
                        new Point(firsthangPose),
                        new Point(23.5,68, Point.CARTESIAN),
                        new Point(readyPose)
                )
        );
        first_hang_back.setLinearHeadingInterpolation(Math.toRadians(0), pickupPose.getHeading());
        first_hang_back.setZeroPowerAccelerationMultiplier(3);
        second_hang = new Path(
                // Line 5
                new BezierCurve(
                        new Point(pickupPose),
                        new Point(15.5, 63, Point.CARTESIAN),
                        new Point(secondhangPose)
                )
        );
        second_hang.setLinearHeadingInterpolation(pickupPose.getHeading(), Math.toRadians(0));
        second_hang.setZeroPowerAccelerationMultiplier(1.25);
        second_hang_back = new Path(
                // Line 6
                new BezierCurve(
                        new Point(secondhangPose),
                        new Point(23.5,68, Point.CARTESIAN),
                        new Point(readyPose)
                )
        );
        second_hang_back.setLinearHeadingInterpolation(Math.toRadians(0), readyPose.getHeading());
        second_hang_back.setZeroPowerAccelerationMultiplier(3);
        third_hang = new Path(
                // Line 7
                new BezierCurve(
                        new Point(pickupPose),
                        new Point(15.5, 63, Point.CARTESIAN),
                        new Point(thirdhangPose)
                )
        );
        third_hang.setLinearHeadingInterpolation(pickupPose.getHeading(), Math.toRadians(0));
        third_hang.setZeroPowerAccelerationMultiplier(1.25);
        third_hang_back = new Path(
                // Line 6
                new BezierCurve(
                        new Point(36.000, 65.000, Point.CARTESIAN),
                        new Point(23.5,68, Point.CARTESIAN),
                        new Point(pickupPose)
                )
        );
        third_hang_back.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(320));
        third_hang_back.setZeroPowerAccelerationMultiplier(3);
        fourth_hang = new Path(
                // Line 7
                new BezierCurve(
                        new Point(13.000, 11.000, Point.CARTESIAN),
                        new Point(14.199, 59.162, Point.CARTESIAN),
                        new Point(36.000, 65.000, Point.CARTESIAN)
                )
        );
        fourth_hang.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0));
        fourth_hang_back = new Path(
                // Line 6
                new BezierCurve(
                        new Point(36.000, 65.000, Point.CARTESIAN),
                        new Point(14.199, 59.162, Point.CARTESIAN),
                        new Point(13.000, 11.000, Point.CARTESIAN)
                )
        );
        fourth_hang_back.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        fourth_hang_back.setZeroPowerAccelerationMultiplier(1.5);
        park = new Path(
                new BezierCurve(
                        new Point(parkPose),
                        new Point(12.070, 59.679, Point.CARTESIAN),
                        new Point(10.393, 24.475, Point.CARTESIAN)
                )
        );
        park.setConstantHeadingInterpolation(0);

        pickup1 = new Path (
                new BezierCurve(
                        new Point(readyPose1),
                        new Point(pickupPose1)
                )
        );
        pickup1.setLinearHeadingInterpolation(readyPose1.getHeading(), pickupPose1.getHeading());
        pickup1.setZeroPowerAccelerationMultiplier(1.5);


    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() function on line 193)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. **/
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 2: //go to hang
                follower.followPath(hang_first);
                setArmState(1); // arm hang pos
                setoutClawState(1); // hang claw pos
                setoutGrabState(4); // unstable outtake state
                setPathState(3);
                break; // BREAK

            case 3: //hang
                if (pathTimer.getElapsedTime() > 2*(Math.pow(10,9))){ // TODO: Time to reach hang Position, shorten
                    setArmState(3);
                    setoutGrabState(4); // unstable outtake state
                    setoutClawState(2);
                    setPathState(4);
                }
                break; // BREAK
            case 4:
                if (pathTimer.getElapsedTime() > (0.7*(Math.pow(10,9)))) { // TODO : Allowing hang time / release
                    setPathState(5);
                }
                break; // BREAK
            case 5: // Starts the push all curve, don't think we need a wait time here
                follower.followPath(pushAll1);
                if(pathTimer.getElapsedTimeSeconds() > 1) {
                    setoutClawState(1); // pickup position claw// Curve forward
                    setArmState(0); // sets arm down
                    setoutGrabState(-1); // Stops grab
                    setPathState(7);
                }
                /// The time frame between the hang and the pushall it is advised to set the servo and arm back in pickup position, however, they must be lowered when directly after the hang itself or else it might latch onto the low bar
                break; // BREAK
            case 7:
                if (!follower.isBusy() || follower.getPose().roughlyEquals(pushstart, 1)) {// await based on distance, calls when its clsoe to behind of first sample
                    follower.followPath(pushAll3); // straight back, ends with first push pose
                    setPathState(8);
                }
                break; // break
            case 8:
                if (!follower.isBusy() || follower.getPose().roughlyEquals(firstpushPose, 1)) { // end of push all 3 into the observation zone doesn't stop and continues
                    //if (/*follower.getPose().getX() > 57 && follower.getPose().getY() > 23*/ !follower.isBusy()) { // curve
                    follower.followPath(pushAll4); // curve forward
                    setPathState(9);
                }
                break; // break
            case 9:
                if (!follower.isBusy() || follower.getPose().roughlyEquals(pushstart2, 1)) { // follower not busy or close to end of the curve forward
                    follower.followPath(pushAll5); // straight back
                    setoutGrabState(3); //grab
                    setPathState(12); //skip pushing third one to save time (very sad)
                }
                break; // BREAK
            // skipped case 10 cuz there was some stuff
            /*case 12:
                if (!follower.isBusy() || follower.getPose().roughlyEquals((endPush))) { // calls in once its at the end of push stage following push all 5
                    /*follower.followPath(ready_pickup);
                    setPathState(13);
                    setoutClawState(3);
                }
                break; // BREAK

            case 13:
                if (!follower.isBusy() || follower.getPose().roughlyEquals((readyPose))) {
                    follower.followPath(pickup);
                    setPathState(14);
                }
                break; // BREAK

                     */
            case 12:
                if(!follower.isBusy() || follower.getPose().roughlyEquals(endPush, 1)) {
                    follower.holdPoint(readyPose);
                    setPathState(13);
                }
            case 13:
                if (pathTimer.getElapsedTime() > (2*Math.pow(10,9))) {
                    follower.followPath(pickup);
                    setPathState(14);
                    setoutGrabState(2);
                }
            case 14:
                if (pathTimer.getElapsedTime() > (2*Math.pow(10,9))) { // TODO pick up time shorten
                    follower.followPath(first_hang);
                    setArmState(1); //up
                    setoutGrabState(4); // unstable release path state
                    setoutClawState(1); // Constant corrections for claw state so i never miss a hang
                    setPathState(145); //145 is equivalent to 14.5 but we cant use double

                }
                break;
            case 145:
                if (pathTimer.getElapsedTime() > (2.2*Math.pow(10,9))) { //TODO: HANG CODE time to reach hang pos, then hang shorten
                    setArmState(3);
                    setoutClawState(2);
                    setPathState(146);
                }
                break;
            case 146:
                if (pathTimer.getElapsedTime() > (0.7*Math.pow(10,9))) { // TODO : Time to release, shorten
                    setPathState(15);
                }
                break;
            case 15:
                //if (!follower.isBusy() || follower.getPose().roughlyEquals(firsthangPose)) { // TODO : see if roughly equals is good enough, i dont think this is needed
                follower.followPath(first_hang_back);
                setoutClawState(3);
                if(pathTimer.getElapsedTime() > (0.5*Math.pow(10,9))) {
                    setArmState(0);
                    setPathState(156);
                }
                break;
            /*case 155:
                if (pathTimer.getElapsedTime() > (3*Math.pow(10,9))) { // TODO time to get out of bar
                    setoutClawState(1);
                    setPathState(156);
                }
                break;

             */
            case 156:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > (1.5 * Math.pow(10, 9))) {
                    follower.followPath(pickup);
                    setPathState(16);
                    setoutGrabState(2);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTime() > (2*Math.pow(10,9))) { // TODO time to reach pickup/pickup
                    // pickup
                    follower.followPath(second_hang);
                    setoutClawState(1);
                    setoutGrabState(4);
                    setArmState(1);
                    setPathState(165);
                }
                break;
            case 165:
                if (pathTimer.getElapsedTime() > (2.2*Math.pow(10,9))) {// TODO : hang
                    setArmState(3);
                    setoutClawState(2);
                    setPathState(166);
                }
                break;
            case 166:
                if (pathTimer.getElapsedTime() > (0.7*Math.pow(10,9))) {// time for relase
                    setPathState(17);
                }
                break;
            case 17:
                follower.followPath(second_hang_back);
                setoutClawState(3);
                if(pathTimer.getElapsedTime() > (0.5*Math.pow(10,9))) {
                    setArmState(0);
                    setPathState(175);
                }
                break;
            case 175:
                if(pathTimer.getElapsedTime() > (2*Math.pow(10,9))) {
                    follower.followPath(pickup);
                    setoutGrabState(2);
                    setPathState(18);
                    setoutClawState(1);
                }
                break;
            case 18:
                if (pathTimer.getElapsedTime() > (1.5*Math.pow(10,9))) { // TODO pickup time
                    follower.followPath(third_hang);
                    setArmState(1);
                    setoutClawState(1);
                    setoutGrabState(4);
                    setPathState(185);
                }
                break;
            case 185:
                if (pathTimer.getElapsedTime() > (2.2*Math.pow(10,9))) { // wait to reach, hang
                    setArmState(3);
                    setoutClawState(2);
                    setPathState(186);
                }
                break;

            case 186:
                if (pathTimer.getElapsedTimeSeconds() > 0.7) { // wait hang, for relase
                    setPathState(19);
                }
                break;
            case 19:
                follower.followPath(third_hang_back);
                setPathState(22);
                break;
            case 22:
                telemetryA.addLine("fucking done.....     oh hi ruben lol");

        }
    }



    /* back_park = follower.pathBuilder()
            .addPath(
            // Line 1
                        new BezierLine(
                    new Point(startPose),
                                new Point(59.660, 84.873, Point.CARTESIAN)
                        )
                                )
                                .setTangentHeadingInterpolation()
                .addPath(
            // Line 2
                        new BezierLine(
                    new Point(59.660, 84.873, Point.CARTESIAN),
                                new Point(10.121, 85.051, Point.CARTESIAN)
                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

     */

    /** This switch is called continuously and runs the necessary actions, when finished, it will set the state to -1.
     * (Therefore, it will not run the action continuously) **/
    public void autonomousActionUpdate() {
        switch (armState) {
            case -1:
                up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//most of the code stolen from opmode_main
            case 0: //going to bottom position
                up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Lowered position", true);
                if (!up_zero.isPressed()) {
                    up.setPower(-1);
                } else if (up_zero.isPressed()) {
                    up.setPower(0);
                    up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                break;
            case 15:
                up.setTargetPosition(150);
                up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                up.setPower(-0.7);
                break;

            case 1: //going to hanging position
                up.setTargetPosition(up_hanging_position);
                up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                up.setPower(1);
                break;
            case 2: //going to hanging position
                up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("hang position 2", true);
                if (up.getCurrentPosition() > up_hanging_position_done) {
                    up.setPower(-0.7);
                    telemetry.addData("arm moving", true);
                } else if (up.getCurrentPosition() <= up_hanging_position_done) {
                    up.setPower(0.01);
                }
                break;
            case 3:
                up.setTargetPosition(1560);
                up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                up.setPower(-0.6);
                break;

        }
        switch (outclawState) {
            case -1:
                servo_outtake_wrist.setPosition(0);
                telemetry.addData("claw position 1 ", true);
                break;
            case 1:
                servo_outtake_wrist.setPosition(0.58);
                servo_intake_wrist.setPosition(0.8);
                telemetry.addData("claw position 2", true);
                break;
            case 2: // Hang done Pos
                servo_outtake_wrist.setPosition(0.23);
                servo_intake_wrist.setPosition(0);
                break;
            case 3:
                servo_outtake_wrist.setPosition(0.47);
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
        /*switch (inclawState) {
            case 0:
                servo_intake_wrist.setPosition(0);
                break;
            case 1:
                servo_intake_wrist.setPosition(0.5);
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

        }*/
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
        out.setTargetPosition(0);
        out.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        out.setPower(1);

        servo_outtake = hardwareMap.get(CRServo.class,"outtake");


        servo_intake = hardwareMap.get(CRServo.class, "intake");



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
        setPathState(2);
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
