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
@Autonomous(name = "Old 4+0", group = "AUTO")
///* This is the Spark invitational auto
/// We won spark 2nd in placement, 1st pick alliance, and also 2nd inspire.
/// This  is a 4+0 autonomous and park for the into the deep season, using bezier curves and linear interpolation, along with a nano-second timer
public class right_auto_old extends OpMode {

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
    private Pose readyPose = new Pose(turn_distance_x,pickups_y, Math.toRadians(180)); // turn spot so make sure it would be safe
    private Pose pickupPose = new Pose(pickup_x, pickups_y, Math.toRadians(180));
    private Pose pickupPose1 = new Pose(pickup_x,18,Math.toRadians(180)); // first pickup poses

    /// hang poses
    private Pose hangPose = new Pose(35, 74, Math.toRadians(0)); // Hits it pretty hard, but its fine
    private Pose firsthangPose = new Pose(36.1,71,0); // Quick hang, then back
    private Pose secondhangPose = new Pose(36.1,65,0); // Should be quick hang, but idk they all mess up around here but somehow everything just works fine
    private Pose thirdhangPose = new Pose(36.3, 72,0); // Last hang

    /// push poses  for case transitions
    private Pose pushstart = new  Pose(59,30,Math.toRadians(180)); // Bezier curve end behind second sample point
    private Pose firstpushPose = new Pose(20,30, Math.toRadians(0)); // ^^ only x matters, keep y same
    private Pose pushstart2 = new Pose(59,18,Math.toRadians(180));  // same

    /// Paths, and path chains : pushFirst and pushSecond are called after hangFirst
    private Path hang_First, pickup, first_hang, first_hang_back, second_hang, second_hang_back, third_hang, park;
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
        hang_First = new Path(
                new BezierLine(
                        new Point(startPose),
                        new Point(hangPose)
                )
        );
        hang_First.setConstantHeadingInterpolation(hangPose.getHeading());
        hang_First.setZeroPowerAccelerationMultiplier(3.5);

        pushFirst = follower.pathBuilder()
                .addPath( new BezierCurve(
                        new Point(hangPose),
                        new Point(30.50989522700815, 18.77532013969732, Point.CARTESIAN),
                        new Point(56.164144353899886, 53.811408614668224, Point.CARTESIAN),
                        new Point(pushstart)
                )
        )
                .setConstantHeadingInterpolation(0)
        .addPath(
                new BezierLine(
                        new Point(pushstart),
                        new Point(firstpushPose)
                )
        )
                .setConstantHeadingInterpolation(0)
                .build();

        pushSecond = follower.pathBuilder()
                .addPath(
                new BezierCurve(
                        new Point(firstpushPose),
                        new Point(48.94994179278231, 36.71245634458673, Point.CARTESIAN),
                        new Point(56.82887077997671, 46.26775320139697),
                        new Point(pushstart)
                )
        )
                .setLinearHeadingInterpolation(firstpushPose.getHeading(),pickupPose1.getHeading())
                .addPath(
                new BezierLine(
                        new Point(pushstart2),
                        new Point(pickupPose1)
                )
        )
        .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3.25)
                .build();

        /// END OF PUSH ALL



        first_hang = new Path(
                // Line 3
                new BezierCurve(
                        new Point(pickupPose1),
                    //    new Point(28.587366694011486, 21.73584905660377, Point.CARTESIAN),
                        new Point(8.623461853978672, 66.15258408531584, Point.CARTESIAN),
                        new Point(firsthangPose)
                )
        );
        first_hang.setLinearHeadingInterpolation(pickupPose.getHeading(), Math.toRadians(0));
        first_hang_back = new Path(
                // Line 4
                new BezierLine(
                        new Point(firsthangPose),
                        new Point(readyPose)
                )
        );
        first_hang_back.setLinearHeadingInterpolation(Math.toRadians(0), readyPose.getHeading());

        pickup = new Path(
                // Line 2
                new BezierLine(
                        new Point(readyPose),
                        new Point(pickupPose)
                )
        );
        pickup.setConstantHeadingInterpolation(Math.toRadians(180));
        pickup.setZeroPowerAccelerationMultiplier(3);

        second_hang = new Path(
                // Line 5
                new BezierCurve(
                        new Point(pickupPose),
                    //    new Point(27.28794093519278, 32.84003281378179, Point.CARTESIAN),
                        new Point(8.26907301066448, 63.78999179655455, Point.CARTESIAN),
                        new Point(secondhangPose)
                )
        );
        second_hang.setLinearHeadingInterpolation(pickupPose.getHeading(), Math.toRadians(0));
        second_hang_back = new Path(
                // Line 6
                new BezierLine(
                        new Point(secondhangPose),
                        new Point(readyPose)
                )
        );
        second_hang_back.setLinearHeadingInterpolation(Math.toRadians(0), readyPose.getHeading());
        third_hang = new Path(
                // Line 7
                new BezierCurve(
                        new Point(pickupPose),
                        // new Point(27.28794093519278, 30.359310910582437, Point.CARTESIAN),
                        new Point(7.914684167350287, 61.191140278917146, Point.CARTESIAN),
                        new Point(thirdhangPose)
                )
        );
        third_hang.setLinearHeadingInterpolation(pickupPose.getHeading(), Math.toRadians(0));
        park = new Path(
                new BezierCurve(
                        new Point(thirdhangPose),
                        new Point(10,24)
                )
        );
        park.setTangentHeadingInterpolation();
        park.setZeroPowerAccelerationMultiplier(4.2);
        park.setReversed(true);




    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 2: //go to hang
                setArmState(111);
                if(pathTimer.getElapsedTime() > (0.5*(Math.pow(10,9)))) {
                    follower.followPath(hang_First, true);
                    setPathState(3);
                }
                break; // BREAK
            case 3: //hang
                if (!follower.isBusy()){ // TODO: Time to reach hang Position, shorten
                    setArmState(3);
                    setoutClawState(2);
                    setoutGrabState(4);
                    setPathState(4);
                }
                break; // BREAK
            case 4:
                if (pathTimer.getElapsedTime() > (0.6*(Math.pow(10,9)))) { // TODO : Allowing hang time / release
                    setPathState(5);
                }
                break; // BREAK
            case 5: // Starts the push all curve, don't think we need a wait time here
                follower.followPath(pushFirst);
                setoutClawState(3);
                setoutGrabState(-1);
                setPathState(81);
                break; // BREAK
            case 81:
                if(pathTimer.getElapsedTimeSeconds() > 1) {
                    setArmState(0);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() || follower.getPose().roughlyEquals(firstpushPose, 1)) { // end of push all 3 into the observation zone doesn't stop and continues
                    follower.followPath(pushSecond); // curve forward
                    setoutGrabState(2);
                    setPathState(13);
                }
                break; // break

            case 13:
                if (!follower.isBusy()) {
                    setPathState(141);
                }
                break;
            case 141:
                if (pathTimer.getElapsedTime() > (0.3*Math.pow(10,9))) { //pickup time
                    setArmState(1);
                        follower.followPath(first_hang);
                        setoutGrabState(4);
                        setPathState(145);
                }
                break;
            case 145:
                if (!follower.isBusy()) { // Instant transition between reaching hang position and hang
                    setArmState(3);
                    setoutClawState(2);
                    setPathState(146);
                }
                break;
            case 146:
                if (pathTimer.getElapsedTime() > (0.6*Math.pow(10,9))) { // Release Time
                    setPathState(15);
                }
                break;
            case 15:
                follower.followPath(first_hang_back);
                setoutClawState(3);
                setPathState(155);
                break;
            case 155:
                if (pathTimer.getElapsedTime() > (0.5*Math.pow(10,9))) { // Time for follower to get out of bar before lowering arm
                    setArmState(0);
                    setoutGrabState(2);

                    setPathState(156);
                }
                break;

            case 156:
                setoutGrabState(2);
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(pickup);
                    setPathState(161);
                }
                break;
            case 161:
                if(!follower.isBusy()) {
                setPathState(162);
                }
            break;
            case 162:
                if (pathTimer.getElapsedTime() > (0.1*Math.pow(10,9))) { // Time of pickup
                    setArmState(1);
                    setoutGrabState(4);
                    setoutClawState(1);
                    // pickup
                    follower.followPath(second_hang, true);
                    setPathState(165);
                }
                break;
            case 165:
                if (!follower.isBusy()) {// TODO : hang
                    setArmState(3);
                    setoutClawState(2);
                    setPathState(166);
                }
                break;
            case 166:
                if (pathTimer.getElapsedTime() > (0.5*Math.pow(10,9))) {// time for relase
                    setPathState(17);
                }
                break;
            case 17:
                follower.followPath(second_hang_back);
                setoutClawState(3);
                setPathState(171);
                break;
            case 171:
                if(pathTimer.getElapsedTime() > (0.5*Math.pow(10,9))) {
                    setArmState(0);
                    setoutGrabState(2);
                    setPathState(175);
                }
                break;
            case 175:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(pickup);
                    setPathState(18);
                }
                break;
            case 18:
                if(!follower.isBusy()) {
                    setoutClawState(0);
                    setPathState(181);
                }
                break;
            case 181:
                if (pathTimer.getElapsedTime() > (0.1*Math.pow(10,9))) { // pickup time
                    follower.followPath(third_hang);
                    setArmState(1);
                    setoutGrabState(4);
                    setPathState(185);
                }
                break;
            case 185:
                if (!follower.isBusy()) { // hang when path done
                    setArmState(3);
                    setoutClawState(2);
                    setPathState(186);
                }
                break;
            case 186:
                if (pathTimer.getElapsedTime() > (0.4*Math.pow(10,9))) { // pickup time
                    setPathState(19);
                }
                break;
            case 19:
                follower.setMaxPower(1);
                follower.followPath(park);
                setPathState(20);
                break;
            case 20:
                if(!follower.isBusy()) {
                    telemetry.addData("hi", true);
                }
                break;

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