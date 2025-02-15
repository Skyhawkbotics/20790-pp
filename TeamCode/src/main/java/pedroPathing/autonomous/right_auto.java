package pedroPathing.autonomous;


import android.os.storage.StorageManager;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.NanoTimer;
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
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import java.net.BindException;

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
    private Pose startPose = new Pose(10, 67.0, Math.toRadians(0)); // TODO - find a good start pos

    /** Hang Poses
     * Used for each of the four hangs, in this case, set 2 inches apart
     * Preload hang Pose is slightly overshoot in case human player doesn't touch the back wall with the robot / field errors
     * The other hang poses should naturally overshoot anyway
     */
    private Pose preloadhangPose = new Pose(36.5, 70, 0); // means it curves left a bit so we can fit more specimens

    private Pose firsthangPose = new Pose(36.0,68,0);

    private Pose secondhangPose = new Pose(36.0,66,0);

    private Pose thirdhangPose = new Pose(36.0, 64,0);


    /** Push Poses
     * End push marks end of push and the transition to pickups
     * first pickup is at the same y value as the end push
     * other pickups are supposed to be at the start of the observation zone
     */
    private Pose pushstart = new  Pose(61.000, 28.000,0); // behind the first one

    private Pose firstpushPose = new Pose(20.000, 28.000, 0); // pushed the first one in

    private Pose pushstart2 = new Pose(63.000, 14.000,0); // behind second

    private Pose endPush = new Pose(20.000, 14.000, Math.toRadians(0)); // pushed second

    /** These are the pickup poses, the pickup paths, and the turn before poses
     * Pickup Pose 1 is for the first hang, picking up from the end push area
     * readyPose1 is a marker for the before the pickup1 and after the pickup1, facing the human player
     * pickupDone1 is after moving back and after the 180 degree turn - either holdPoint or the new turn() method
     */
    private Pose readyPose1 = new Pose(20,14, Math.toRadians(180));
    private Pose pickupPose1 = new Pose(6 ,14,Math.toRadians(180));
    private Pose pickupDone1 = new Pose(20,14, Math.toRadians(0));


    private Pose before_ready = new Pose(20,37, 0); // Ready pose before the turn

    /// There is a 180 degree turn between these two
    private Pose readyPose = new Pose(20,37, Math.toRadians(180)); // these two have to have same y cords

    private Pose pickupPose = new Pose( 6, 37, Math.toRadians(180)); // ^^^

    /// Finally, the part Pose

    private Pose parkPose = new Pose(13.703, 20.673, Point.CARTESIAN);


    /// Paths - in order of calling, separated into 3 segments
    // Preload Path, and the two Pushes

    private Path hang_first, pushAll1, pushAll3, pushAll4, pushAll5;
    // The First pick up and hang then transition to the next pickup spot
    private Path pickup1, pickup1_back, first_hang, first_hang_back;
    // The other hang and pickup spots
    private Path pickup, pickup_back, second_hang, second_hang_back, third_hang;
    private Path park;


    /// Motors
    private DcMotorEx up, out;
    private Servo servo_outtake_wrist, servo_intake_wrist, servo_intake_rotate;
    private CRServo servo_outtake, servo_intake;
    private TouchSensor up_zero, out_zero;
    private Telemetry telemetryA;

    double pickup_time = 1.5;


    /// Values for the hangs
    int up_hanging_position = 1775; //DONE: calibrate this value, viper slide position to
    int up_hanging_position_done = 1270; //TODO: calibrate this value, position of viper slide when releasing after speciman is on the bar.
    // 1543
    //0.29
    // also if we put it too low maybe it might drift when it stretches
    public void buildPaths() {
        hang_first = new Path(
                new BezierLine(
                        new Point(startPose),
                        new Point(preloadhangPose)
                )
        );
        hang_first.setConstantHeadingInterpolation(preloadhangPose.getHeading());
        hang_first.setZeroPowerAccelerationMultiplier(1.5);

        pushAll1 = new Path(
                new BezierCurve(
                        new Point(preloadhangPose),
                        new Point(22.100, 20.000, Point.CARTESIAN),
                        new Point(22.100, 20.000, Point.CARTESIAN),
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
        pushAll3.setConstantHeadingInterpolation(0);
        pushAll3.setZeroPowerAccelerationMultiplier(1.5);
        pushAll4 = new Path (
                new BezierCurve(
                        new Point(firstpushPose),
                        new Point(53.631, 28.942, Point.CARTESIAN),
                        new Point(pushstart2)
                )
        );
        pushAll4.setConstantHeadingInterpolation(0); // curve toget in front of second sample
        pushAll5 = new Path(
                new BezierLine(
                        new Point(pushstart2),
                        new Point(endPush)
                )
        );
        pushAll5.setConstantHeadingInterpolation(0);
        pushAll5.setZeroPowerAccelerationMultiplier(1.5);

        /// Turn 180 here


        pickup1 = new Path (
                // Line 3
                new BezierLine(
                        new Point(readyPose1),
                        new Point(pickupPose1)
                )
        );
        pickup1.setConstantHeadingInterpolation(Math.toRadians(180));;
        pickup1.setZeroPowerAccelerationMultiplier(1.5);

        pickup1_back = new Path(
                // Line 4
                new BezierLine(
                        new Point(pickupPose1),
                        new Point(readyPose1)
                )
        );
        pickup1_back.setConstantHeadingInterpolation(Math.toRadians(180));
        pickup1_back.setZeroPowerAccelerationMultiplier(1.25);

        /// Turn 180 here


        first_hang = new Path(
                // Line 5
                new BezierCurve(
                        new Point(pickupDone1),
                        new Point(15.357, 64.144, Point.CARTESIAN),
                        new Point(firsthangPose)
                )
        );
        first_hang.setConstantHeadingInterpolation(Math.toRadians(0));
        first_hang.setZeroPowerAccelerationMultiplier(1.25);
        first_hang_back = new Path( // strafe to secondary pick pose
                // Line 6
                new BezierLine(
                        new Point(firsthangPose),
                        new Point(before_ready)
                )
        );
        first_hang_back.setConstantHeadingInterpolation(Math.toRadians(0));
        first_hang_back.setZeroPowerAccelerationMultiplier(1.25);

        /// turn 180

        pickup = new Path( // Will be referenced for each pick up
                // Line 7
                new BezierLine(
                        new Point(readyPose),
                        new Point(pickupPose)
                )
        );
        pickup.setConstantHeadingInterpolation(Math.toRadians(180));
        pickup.setZeroPowerAccelerationMultiplier(1.25);
        pickup_back = new Path( // Referenced after each pick up
                // Line 8
                new BezierLine(
                        new Point(pickupPose),
                        new Point(readyPose)
                )
        );
        pickup_back.setConstantHeadingInterpolation(Math.toRadians(180));
        pickup_back.setZeroPowerAccelerationMultiplier(1.25);

        /// Turn 180 here

        second_hang = new Path(
                // Line 9
                new BezierCurve(
                        new Point(before_ready),
                        new Point(15.711, 59.774, Point.CARTESIAN),
                        new Point(secondhangPose)
                )
        );
        second_hang.setConstantHeadingInterpolation(Math.toRadians(0));
        second_hang.setZeroPowerAccelerationMultiplier(1.25);
        second_hang_back = new Path(
                // Line 10
                new BezierLine(
                        new Point(secondhangPose),
                        new Point(before_ready)
                )
        );
        second_hang_back.setConstantHeadingInterpolation(Math.toRadians(0));
        second_hang_back.setZeroPowerAccelerationMultiplier(1.25);
        // Turn 180 here

        /// Run pickup, and pickup_back

        // Turn 180 again

        third_hang = new Path(
                // Line 13
                new BezierCurve(
                        new Point(before_ready),
                        new Point(16.774, 55.639, Point.CARTESIAN),
                        new Point(thirdhangPose)
                )
        );
        third_hang.setConstantHeadingInterpolation(Math.toRadians(0));
        third_hang.setZeroPowerAccelerationMultiplier(1.25);
        park = new Path(
                // Line 14
                new BezierLine(
                        new Point(thirdhangPose),
                        new Point(parkPose)
                )
        );
        park.setTangentHeadingInterpolation();

    }

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
                    setoutClawState(4); // pickup position claw// Curve forward
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
                    // grab
                    setPathState(12); //skip pushing third one to save time (very sad)
                }
                break; // BREAK
// Turn here
            case 10:
                if(!follower.isBusy() || follower.getPose().roughlyEquals(endPush, 1)) {
                    follower.turnTo(Math.toRadians(180));
                    setoutGrabState(3);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy() || follower.getPose().roughlyEquals(readyPose1, 1) /*|| pathTimer.getElapsedTime() >(1.5*Math.pow(10,9))*/) { // wait time for turn
                    setPathState(13);
                }
                break;
            case 13:
                follower.followPath(pickup1);
                setPathState(14);
                break;
            case 14:
                if (pathTimer.getElapsedTime() > (1.5*Math.pow(10,9))) { /// Time for Path for pickup and Pickup time
                    setArmState(4); //up above wall
                    setoutGrabState(4); // unstable release path state
                    setoutClawState(4); // Constant corrections for claw state so i never miss a hang
                    setPathState(141); //145 is equivalent to 14.5 but we cant use double

                }
                break;
            case 141:
                if (pathTimer.getElapsedTime() > (1.5*Math.pow(10,9))) { /// Time for arm to clear the wall todo calibrate this
                    follower.followPath(pickup1_back);
                    setPathState(142);
                }
                break;
            case 142:
                if(pathTimer.getElapsedTime() > (3*Math.pow(10,9)) || !follower.isBusy() || follower.getPose().roughlyEquals(readyPose1)) { /// Idk time to finish path pickup1 back todo calibrate
                    follower.turnTo(Math.toRadians(0)); // turns to face hangs
                    setPathState(143);
                }
                break;
            case 143:
                if(pathTimer.getElapsedTime() > (3*Math.pow(10,9)) || !follower.isBusy() || follower.getPose().roughlyEquals(pickupDone1)) { // time to turn todo calibrate
                    follower.followPath(first_hang);
                    setArmState(2);// hang pos
                    setPathState(145);
                }
                break;
            case 145:
                if (pathTimer.getElapsedTime() > (3*Math.pow(10,9))  || follower.getPose().roughlyEquals(firsthangPose, 0.2) ) { //todo time to reach hang pos
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
                follower.followPath(first_hang_back);
                setoutClawState(3);
                if(pathTimer.getElapsedTime() > (0.5*Math.pow(10,9))) {
                    setArmState(0); // all the way down
                    setoutClawState(3); // pickup pos
                    setPathState(151);
                }
                break;
            case 151:
                if (pathTimer.getElapsedTime() > (3*Math.pow(10,9))  || follower.getPose().roughlyEquals(before_ready, 1)) { // time to reach before ready - todo calibrate this down
                    follower.turnTo(Math.toRadians(180));
                    setPathState(156);
                }
                break;
            case 156:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > (1.5 * Math.pow(10, 9)) || follower.getPose().roughlyEquals(readyPose, 1)) { // time to turn todo calibrate
                    setoutGrabState(2);
                    follower.followPath(pickup);
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTime() > (1.5*Math.pow(10,9))) { // TODO time to reach pickup/pickup
                    setoutGrabState(4);
                    setArmState(4);
                    setPathState(161);
                }
                break;
            case 161:
                if (pathTimer.getElapsedTime() > (1.5*Math.pow(10,9))) { // TODO time to raise the viper slide
                    follower.followPath(pickup_back);
                    setPathState(162);
                }
                break;
            case 162:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > (1.5 * Math.pow(10, 9)) || follower.getPose().roughlyEquals(readyPose, 1)) {// todo time to reach ready pose for turn
                    follower.turnTo(Math.toRadians(0));
                    setPathState(163);
                }
                break;
            case 163:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > (1.5 * Math.pow(10, 9)) || follower.getPose().roughlyEquals(before_ready, 1)) { // time to turn todo calibrate
                    setArmState(2);
                    setoutClawState(4);
                    follower.followPath(second_hang);
                    setPathState(165);
                }
                break;
            case 165:
                if (pathTimer.getElapsedTime() > (2.2*Math.pow(10,9))  || follower.getPose().roughlyEquals(secondhangPose, 0.2)) {// TODO : hang
                    setArmState(3);
                    setoutClawState(2);
                    setPathState(166);
                }
                break;
            case 166:
                if (pathTimer.getElapsedTime() > (0.7*Math.pow(10,9))) {// time for release
                    setPathState(17);
                }
                break;
            case 17:
                follower.followPath(second_hang_back);
                setoutClawState(3);
                if(pathTimer.getElapsedTime() > (0.5*Math.pow(10,9))) {
                    setArmState(0);
                    setPathState(171);
                }
                break;
            case 171:
                if (pathTimer.getElapsedTime() > (3*Math.pow(10,9))  || follower.getPose().roughlyEquals(before_ready, 1)) { // time to reach before ready - todo calibrate this down
                    follower.turnTo(Math.toRadians(180));
                    setPathState(175);
                }
                break;
            case 175:
                if(pathTimer.getElapsedTime() > (1.5*Math.pow(10,9)) || follower.getPose().roughlyEquals(readyPose, 1)) { // time to turn to start pickup todo
                    setoutGrabState(2);
                    follower.followPath(pickup);
                    setPathState(18);
                    setoutClawState(4);
                }
                break;
            case 18:
                if (pathTimer.getElapsedTime() > (1.5*Math.pow(10,9))) { // TODO pickup time / to reach pickup spot;
                    setArmState(4);
                    setoutClawState(4);
                    setoutGrabState(4);
                    setPathState(181);
                }
                break;
            case 181:
                if (pathTimer.getElapsedTime() > (1.5*Math.pow(10,9))) { // TODO time to raise the viper slide
                    follower.followPath(pickup_back);
                    setPathState(182);
                }
                break;
            case 182:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > (1.5 * Math.pow(10, 9)) || follower.getPose().roughlyEquals(readyPose, 1)) {// todo time to reach ready pose for turn
                    follower.turnTo(Math.toRadians(0));
                    setPathState(183);
                }
                break;
            case 183:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > (1.5 * Math.pow(10, 9)) || follower.getPose().roughlyEquals(before_ready, 1)) { // time to turn todo calibrate
                    setArmState(2);
                    setoutClawState(4);
                    follower.followPath(second_hang);
                    setPathState(184);
                }
                break;
            case 184:
                if (pathTimer.getElapsedTime() > (2.2*Math.pow(10,9))  || follower.getPose().roughlyEquals(secondhangPose, 0.2)) {// TODO : hang
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
                follower.followPath(park);
                setPathState(22);
                break;
            case 22:
                telemetryA.addLine(".....Done....");

        }
    }

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
            case 4: // Higher than the wall
                up.setTargetPosition(300); // todo calibrate
                up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                up.setPower(1);

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
                servo_outtake_wrist.setPosition(0.25);
                servo_intake_wrist.setPosition(0);
                break;
            case 3: // pickup? todo calibrate
                servo_outtake_wrist.setPosition(0.47);
                break;
            case 4:
                servo_outtake_wrist.setPosition(0.58);

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
            case 4: // unstable state
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
