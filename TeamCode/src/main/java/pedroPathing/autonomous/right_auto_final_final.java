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
// 18.5 inches away from observation zone
public class right_auto_final_final extends OpMode {

    private Follower follower; // THe drivetrain with the calculations needed for path following
    private Timer actionTimer, opmodeTimer, outtimer; // Timers for progression of states

    private NanoTimer pathTimer;
    double pickup_x = 0.6;

    double first_pickup_y = 20;

    double turn_distance_x = 20;

    double pickups_y = 41.00;


    private int pathState, armState, outclawState, outgrabState, inclawState, ingrabState; // Different cases and states of the different parts of the robot

    private Pose startPose = new Pose(10, 67.0, Math.toRadians(0)); //TODO

    private Pose readyPose = new Pose(turn_distance_x,pickups_y, Math.toRadians(180)); /// turn spot so make sure it would be safe

    private Pose pickupPose = new Pose(pickup_x, pickups_y, Math.toRadians(180));
    private Pose readyPose1 = new Pose(25,18, Math.toRadians(180)); /// first pickup poses

    private Pose pickupPose1 = new Pose(pickup_x,18,Math.toRadians(180)); /// first pickup poses


    /// hang poses
    private Pose hangPose = new Pose(35, 74, Math.toRadians(0)); // TODO runs on

    private Pose firsthangPose = new Pose(36.1,71,0);

    private Pose secondhangPose = new Pose(36.1,65,0);

    private Pose thirdhangPose = new Pose(36.3, 72,0);


    /// push poses  for case transitions
    private Pose pushstart = new  Pose(59,30,Math.toRadians(180)); // has to match

    private Pose firstpushPose = new Pose(20,30, Math.toRadians(0)); // ^^

    private Pose pushstart2 = new Pose(59,18,Math.toRadians(180));

    private Pose endPush = new Pose(20,20, Math.toRadians(0)); /// :D




    // Paths
    private PathChain hang1;

    private Path hang_first, park;
    private Path pushAll1, pushAll2, pushAll3, pushAll4, pushAll5, pushAll6, pushAll7, pushAll8, pickup1;

    private Path ready_pickup, pickup, first_hang, first_hang_back, second_hang, second_hang_back, third_hang, third_hang_back, fourth_hang, fourth_hang_back;

    private PathChain pushFirst, pushSecond;

    // Motors
    private DcMotorEx up, out;
    private Servo servo_outtake_wrist, servo_intake_wrist, servo_intake_rotate;
    private CRServo servo_outtake, servo_intake;
    private TouchSensor up_zero, out_zero;
    private Telemetry telemetryA;

    // variables
    double intake_wrist_pos_transfer = 0;
    double outtake_wrist_pos_transfer = 0;
    int up_hanging_position = 1778; //DONE: calibrate this value, viper slide position to
    int up_hanging_position_done = 1250; //TODO: calibrate this value, position of viper slide when releasing after speciman is on the bar.
    // 1543
    //0.29

    public void buildPaths() {
        hang_first = new Path(
                new BezierLine(
                        new Point(startPose),
                        new Point(hangPose)
                )
        );
        hang_first.setConstantHeadingInterpolation(hangPose.getHeading());
        hang_first.setZeroPowerAccelerationMultiplier(3.5);

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
        third_hang_back = new Path(
                // Line 6
                new BezierCurve(
                        new Point(thirdhangPose),
                        new Point(20,72,0)
                )
        );
        third_hang_back.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(320));



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
                    follower.followPath(hang_first, true);
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
                if (!follower.isBusy()) { //TODO: HANG CODE time to reach hang pos, then hang shorten
                    setArmState(3);
                    setoutClawState(2);
                    setPathState(146);
                }
                break;
            case 146:
                if (pathTimer.getElapsedTime() > (0.6*Math.pow(10,9))) { // TODO : Time to release, shorten
                    setPathState(15);
                }
                break;
            case 15:
                //if (!follower.isBusy() || follower.getPose().roughlyEquals(firsthangPose)) { // TODO : see if roughly equals is good enough, i dont think this is needed
                follower.followPath(first_hang_back);
                setoutClawState(3);
                setPathState(155);
                break;
            case 155:
                if (pathTimer.getElapsedTime() > (0.5*Math.pow(10,9))) { // TODO time to get out of bar
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
                if (pathTimer.getElapsedTime() > (0.1*Math.pow(10,9))) { // TODO time to reach pickup/pickup
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
                    setPathState(181);
                }
                break;
            case 181:
                if (pathTimer.getElapsedTime() > (0.1*Math.pow(10,9))) { // TODO pickup time
                    follower.followPath(third_hang);
                    setArmState(1);
                    setoutClawState(1);
                    setoutGrabState(4);
                    setPathState(185);
                }
                break;
            case 185:
                if (!follower.isBusy()) { // wait to reach, hang
                    setArmState(3);
                    setoutClawState(2);
                    setPathState(186);
                }
                break;
            case 186:
                if (pathTimer.getElapsedTime() > (0.4*Math.pow(10,9))) { // TODO pickup time
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
                case -1:
                    up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    break;
//most of the code stolen from opmode_main
                case 0: //going to bottom position
                    up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    telemetry.addData("Lowered position", true);
                    if (!up_zero.isPressed()) {
                        up.setPower(-1);
                    } else if (up_zero.isPressed()) {
                        setArmState(-1);
                    }
                    break;
                case 1: //going to hanging position
                    up.setTargetPosition(up_hanging_position);
                    up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    up.setPower(1);
                    if(up.getCurrentPosition() > 400) {
                        setoutClawState(1);
                    }
                    break;
                case 111: //going to hanging position
                    up.setTargetPosition(1715);
                    up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    up.setPower(1);
                    if(up.getCurrentPosition() > 200) {
                        setoutClawState(1);
                    }
                    break;
                case 2:
                    up.setTargetPosition(150);
                    up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    up.setPower(0.7);
                    break;

                case 3:
                    up.setTargetPosition(1559);
                    up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    up.setPower(0.6);
                    break;

            }
        switch (outclawState) {
            case -1:
                servo_outtake_wrist.setPosition(0);
                telemetry.addData("claw position 1 ", true);
                break;
            case 1:
                servo_outtake_wrist.setPosition(0.58);
                telemetry.addData("claw position 2", true);
                break;
            case 2: // Hang done Pos
                servo_outtake_wrist.setPosition(0.27);
                break;
            case 3:
                servo_outtake_wrist.setPosition(0.55);
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
        int charles = out.getCurrentPosition();
        out.setTargetPosition(charles);
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
