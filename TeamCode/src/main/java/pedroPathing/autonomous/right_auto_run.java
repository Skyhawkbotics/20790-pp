package pedroPathing.autonomous;




import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;


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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.robotcore.external.Telemetry;


import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;




@Config
@Autonomous(name = "RIGHT AUTO RUN ON", group = "AUTO")
/// 5+0 for state
public class right_auto_run extends OpMode {


    private Follower follower;
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;


    /// All motors / servos / sensors
    private Servo servo_outtake_flip1, servo_outtake_flip2, servo_intake_wrist, servo_intake_rotate, servo_intake, servo_outtake, servo_outtake_rotate;
    private DcMotorEx up1, up2, out;
    private TouchSensor up_zero, out_zero;


    // Timers
    private Timer opmodeTimer;
    private NanoTimer pathTimer;
    double pickup_x = 0.6;


    int charles;
    double turn_distance_x = 20; // Distance to start pickup
    double pickups_y = 41.00; // Observation Zone pickup distance
    private int pathState, armState, outclawState, outgrabState, stoneState; // Different cases and states of the different parts of the robot


    // Poses, and pickUp Poses
    private Pose startPose = new Pose(10.000, 55.000, 0); //TODO


    /// hang poses, Make sure they running on hard enough (x), far enough from each other (y)
    private Pose inithangPose = new Pose(37.000, 65.000, Point.CARTESIAN);
    private Pose firsthangPose = new Pose(37, 73.000, Point.CARTESIAN);
    private Pose secondhangPose = new Pose(37, 69.000, Point.CARTESIAN);
    private Pose thirdhangPose = new Pose(37, 67.000, Point.CARTESIAN); // changed from 69
    private Pose fourthhangPose = new Pose(37,65,Point.CARTESIAN);




    /// push poses  for case transitions, pushPoses are when the sample is fully pushed, before is behind the sample
    /// Adjust distance from observation zone (X)
    /// make sure (y) is same as before push
    private Pose curve2pushPose = new Pose(61, 26, 0); // turn spot so make sure it would be safe
    private Pose push1 = new  Pose(35, 26, Point.CARTESIAN);
    private Pose push2 = new Pose(35, 16, Point.CARTESIAN);
    private Pose pickupPose1 = new Pose(35, 8 , 0); // first pickup poses also doubles as end of push 3


    /// adjust (X) to minimize overshoot , (y) to target the sample fully to push , make sure Y is same as push poses
    private Pose beforepush2 = new Pose(61, 16, Point.CARTESIAN);
    private Pose beforepush3 = new Pose(59, 8, Point.CARTESIAN);


    /// Paths, and path chains : pushFirst and pushSecond are called after hangFirst
    private Path init_hang, curve2push, first_hang, first_hang_back, second_hang, second_hang_back, third_hang, third_hang_back, fourth_hang, fourth_hang_back, back, back2, back3;
    private Path pickup;
    private PathChain pushall;


    private Telemetry telemetryA;


    boolean outisclosed = false;


    boolean inisclosed = false;
    public void buildPaths() {
        init_hang = new Path(
                /// init hang path
                new BezierLine(
                        new Point(startPose),
                        new Point(inithangPose)
                )
        );
        init_hang.setConstantHeadingInterpolation(Math.toRadians(0));
        curve2push = new Path(
                /// behind first, make sure this doesn't hit the sub
                new BezierCurve(
                        new Point(inithangPose),
                        new Point(25.9837019790454, 18.607683352735737, Point.CARTESIAN),
                        new Point(55.82305005820722, 39.56228172293365, Point.CARTESIAN),
                        new Point(curve2pushPose)
                )
        );
        curve2push.setConstantHeadingInterpolation(Math.toRadians(0));
        pushall = follower.pathBuilder()
                .addPath(
                        /// push first
                        new BezierLine(
                                new Point(curve2pushPose),
                                new Point(push1)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        /// behind second sample
                        new BezierCurve(
                                new Point(push1),
                                new Point(beforepush2)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(3.9)

                .addPath(
                        /// pushes second
                        new BezierLine(
                                new Point(beforepush2),
                                new Point(push2)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        /// gets behind third
                        new BezierCurve(
                                new Point(push2),
                                new Point(60,15,Point.CARTESIAN),
                                new Point(beforepush3)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(  /// Pushes third
                        // Line 7
                        new BezierLine(
                                new Point(beforepush3),
                                new Point(pickupPose1)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Point(pickupPose1),
                                new Point(35, 16.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(0)
                .addPath(
                        new BezierLine(
                                new Point(35,16.000,Point.CARTESIAN),
                                new Point(11,16.000,Point.CARTESIAN)
                        ))
                .setConstantHeadingInterpolation(0)
                .setZeroPowerAccelerationMultiplier(2)
                .build();









        /// First hang
        first_hang = new Path(
                // Line 1
                new BezierCurve(
                        new Point(11,16),
                        new Point(firsthangPose)
                )
        );
        first_hang.setConstantHeadingInterpolation(Math.toRadians(0));
        back = new Path(
                new BezierLine(
                        new Point(firsthangPose),
                        new Point(37, 73, Point.CARTESIAN)
                )
        );
        back.setConstantHeadingInterpolation(0);
        first_hang_back = new Path(
                // Line 2
                new BezierLine(
                        new Point(firsthangPose),
                        new Point(20.000, 36.000, Point.CARTESIAN)
                )
        );
        first_hang_back.setConstantHeadingInterpolation(Math.toRadians(0));
        pickup = new Path(
                // Line 3
                new BezierLine(
                        new Point(20.000, 36.000, Point.CARTESIAN),
                        new Point(11, 36.000, Point.CARTESIAN)
                )
        );
        pickup.setConstantHeadingInterpolation(Math.toRadians(0));
        second_hang = new Path(
                // Line 4
                new BezierLine(
                        new Point(11.000, 36.000, Point.CARTESIAN),
                        new Point(secondhangPose)
                )
        );
        second_hang.setConstantHeadingInterpolation(Math.toRadians(0));
        back2 = new Path(
                new BezierLine(
                        new Point(secondhangPose),
                        new Point(37,71, Point.CARTESIAN)
                )
        );
        back.setConstantHeadingInterpolation(0);
        second_hang_back = new Path(
                // Line 5
                new BezierLine(
                        new Point(37,71),
                        new Point(20.000, 36.000, Point.CARTESIAN)
                )
        );
        second_hang_back.setConstantHeadingInterpolation(Math.toRadians(0));
        /// Run pickup
        third_hang = new Path(
                // Line 7
                new BezierLine(
                        new Point(11.000, 36.000, Point.CARTESIAN),
                        new Point(thirdhangPose)
                )
        );
        third_hang.setConstantHeadingInterpolation(Math.toRadians(0));
        back3 = new Path(
                new BezierLine(
                        new Point(thirdhangPose),
                        new Point(37,69, Point.CARTESIAN)
                )
        );
        back3.setConstantHeadingInterpolation(0);
        third_hang_back = new Path(
                // Line 8
                new BezierLine(
                        new Point(37,69,Point.CARTESIAN),
                        new Point(20.000, 36.000, Point.CARTESIAN)
                )
        );
        third_hang_back.setConstantHeadingInterpolation(Math.toRadians(0));
        /// Run pickup


        fourth_hang = new Path(
                // Line 10
                new BezierLine(
                        new Point(11.000, 36.000, Point.CARTESIAN),
                        new Point(fourthhangPose)
                )
        );
        fourth_hang.setConstantHeadingInterpolation(Math.toRadians(0));
        fourth_hang_back = new Path(
                // Line 11
                new BezierLine(
                        new Point(fourthhangPose),
                        new Point(11, 36, Point.CARTESIAN)
                )
        );
        fourth_hang_back.setConstantHeadingInterpolation(Math.toRadians(0));
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case -1: // empty case for testing
                // setArmState();
                break;
            case 0:
                setArmState(15); // Ready pos, arm is fully normally aligned. make sure it is near the edge
                setPathState(1);
                break;
            case 1:
                if (pathTimer.getElapsedTime() > (0.2 * (Math.pow(10, 9)))) { // wait time for release
                    follower.followPath(init_hang);
                    setPathState(101);
                }
                break;
            case 101: /// Hang
                if (!follower.isBusy()) {
                    setoutClawState(1); // release
                    setPathState(203);
                }
                break;
            case 203:
                if (pathTimer.getElapsedTime() > (0.25 * (Math.pow(10, 9)))) { // wait time for release
                    follower.followPath(curve2push);
                    setPathState(204);
                }
                break;
            case 204:
                setArmState(0);
                setPathState(2); // do i need to add a wait time here

                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(pushall);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) { // grab
                    setoutClawState(0);
                    setPathState(401);
                }
                break;
            case 401:
                if (pathTimer.getElapsedTime() > (0.5 * (Math.pow(10, 9)))) { // Time to grab
                    setArmState(1);
                    setPathState(402);
                }
                break;
            case 402:
                if (pathTimer.getElapsedTime() > (0.2 * (Math.pow(10, 9)))) { // Time to grab
                    follower.followPath(first_hang, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) { // if it is done
                    setArmState(2);
                    //follower.followPath(back);
                    setPathState(5011);
                }
                break;
            case 5011:
                if (pathTimer.getElapsedTime() > (1 * (Math.pow(10, 9)))) { // Time to release
                    setoutClawState(1);
                    //follower.followPath(first_hang_back);
                    setPathState(501);

                }
                break;

            case 501:
                if (pathTimer.getElapsedTime() > (0.2 * (Math.pow(10, 9)))) { // Time to get out of direciton of specimen
                    follower.followPath(first_hang_back);
                    setArmState(0);
                    setPathState(5);

                }
                break;

            case 5:
                if (!follower.isBusy()) { // waits for stop
                    follower.followPath(pickup);
                    setPathState(601);
                }
                break;
            case 601:
                if (!follower.isBusy()) { // waits for stop
                    setoutClawState(0);
                    setPathState(602);
                }
                break;
            case 602:
                if (pathTimer.getElapsedTime() > (0.4 * (Math.pow(10, 9)))) { // time to grab
                    setArmState(22);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTime() > (0.2 * (Math.pow(10, 9)))) { // time to raise arm
                    follower.followPath(second_hang, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) { // waits for until its at position
                    setoutClawState(1); // lets go
                    setPathState(801);
                }
                break;
            case 801:
                if (pathTimer.getElapsedTime() > (0.4 * (Math.pow(10, 9)))) {// time to relase
                    follower.followPath(second_hang_back);
                    setPathState(802);
                }
                break;
            case 802:
                if (pathTimer.getElapsedTime() > (0.1 * (Math.pow(10, 9)))) { // time to get out
                    setArmState(0);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(pickup);
                    setPathState(901);
                }
                break;
            case 901:
                if (!follower.isBusy()) {
                    setoutClawState(0);
                    setPathState(902);
                }
                break;
            case 902:
                if (pathTimer.getElapsedTime() > (0.5 * (Math.pow(10, 9)))) { // time to close
                    setArmState(1);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTime() > (0.2 * (Math.pow(10, 9)))) { // time to raise arm
                    follower.followPath(third_hang, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    setoutClawState(1);
                    setPathState(90);
                }
                break;
            case 90:
                if (pathTimer.getElapsedTime() > (0.4 * (Math.pow(10, 9)))) { // time to hang
                    follower.followPath(third_hang_back);
                    setPathState(111);
                }
                break;
            case 111:
                if (pathTimer.getElapsedTime() > (0.2 * (Math.pow(10, 9)))) { // time to get out of sub
                    setArmState(0);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) { // wait for stop
                    follower.followPath(pickup);
                    setPathState(121);
                }
                break;
            case 121:
                if (!follower.isBusy()) { // wait for stop
                    setoutClawState(0);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTime() > (0.4 * (Math.pow(10, 9)))) { // time to pick up
                    setArmState(1);
                    setPathState(131);
                    break;
                }
            case 131:
                if (pathTimer.getElapsedTime() > (0.4 * (Math.pow(10, 9)))) { // time to get arm up
                    follower.followPath(fourth_hang, true);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    setoutClawState(1);
                    setPathState(133);
                }
                break;
            case 133:
                if (pathTimer.getElapsedTime() > (0.4 * (Math.pow(10, 9)))) { // time to get arm up
                    follower.followPath(fourth_hang_back);
                    setPathState(144);
                }
                break;
        }
    }
    public void autonomousActionUpdate() {
        switch (armState) {
            case -1: // default stop
                break;
            case 0: // Zero Pos / pickup pos
                servo_outtake_flip2.setPosition(1);
                servo_outtake_flip1.setPosition(0);
                if(servo_outtake_flip1.getPosition() < 0.5) {
                    servo_outtake_rotate.setPosition(0.165);
                }
                break;
            case 1: // Run on position
                servo_outtake_flip2.setPosition(0.58);
                servo_outtake_flip1.setPosition(0.42);
                if(servo_outtake_flip1.getPosition() > 0.1) {
                    servo_outtake_rotate.setPosition(0.805); //set off from center (.805) so its crooked and runs on easier
                }
                break;
            case 15: // Run on position
                servo_outtake_flip2.setPosition(0.58);
                servo_outtake_flip1.setPosition(0.42);
                if(servo_outtake_flip1.getPosition() > 0.1) {
                    servo_outtake_rotate.setPosition(0.805); //set off from center (.805) so its crooked and runs on easier
                }
                break;
            case 2: // TODO TODO TODODO TODO DODO
                servo_outtake_flip2.setPosition(0.4);
                servo_outtake_flip1.setPosition(0.6);
                servo_outtake_rotate.setPosition(0.825); // fully striaght
                break;
        }
        switch (outclawState) {
            case -1: // Init Pos
                break;
            case 0: // closed
                servo_outtake.setPosition(0.25);
                break;
            case 1: // open
                servo_outtake.setPosition(1);
                break;




        }
        switch (stoneState) {
            case 1:
                out.setPower(-1);
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
    public void setoutstate(int lState) {
        armState = lState;
    }
    public void setstonestate(int gstate) {
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

        telemetryA.addData("path state", pathState);
        telemetryA.addData("arm state", armState);
        telemetryA.addData("pathtimer elapsed time", pathTimer.getElapsedTimeSeconds());



        // Poses, follower error
        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData("heading", follower.getPose().getHeading());
        telemetryA.addData("Follower busy", follower.isBusy());
        telemetryA.addData("headingPID error", follower.headingError);
        telemetryA.update();
        out.setPower(-0.1);
    }








    // We init the timers, telemetry, motors , and follower
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);


        pathTimer = new NanoTimer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);




        out = hardwareMap.get(DcMotorEx.class, "out");
        out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        out.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        out.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        out.setDirection(DcMotorSimple.Direction.REVERSE);


        //from rr version

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

        //setup arm to use velocity
        //setup arm variable
        up1 = hardwareMap.get(DcMotorEx.class, "up1");
        up1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up1.setDirection(DcMotorSimple.Direction.REVERSE);
        up1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        up2 = hardwareMap.get(DcMotorEx.class, "up2");
        up2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        servo_intake = hardwareMap.get(Servo.class, "intake");
        servo_intake_rotate = hardwareMap.get(Servo.class, "intakeRotate");
        servo_intake_wrist = hardwareMap.get(Servo.class, "intakeWrist");




        servo_outtake_flip1 = hardwareMap.get(Servo.class, "ofip1");
        servo_outtake_flip2 = hardwareMap.get(Servo.class, "ofip2");


        servo_outtake = hardwareMap.get(Servo.class, "outtake");
        servo_outtake_rotate = hardwareMap.get(Servo.class, "outtaker");
        servo_outtake.setPosition(0.3);
        servo_outtake_flip2.setPosition(0.9);
        servo_outtake_flip1.setPosition(0.1);
        servo_outtake_rotate.setPosition(0.805);

        up_zero = hardwareMap.get(TouchSensor.class, "up_zero");


        out_zero = hardwareMap.get(TouchSensor.class, "out_zero");





        inisclosed = false;


        outisclosed = true;




    }
    @Override
    public void start() {
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    @Override
    public void stop() {
    }
}
