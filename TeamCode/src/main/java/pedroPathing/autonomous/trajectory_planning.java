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
@Autonomous(name = "right traj", group = "AUTO")
public class trajectory_planning extends OpMode {

    private Follower follower; // THe drivetrain with the calculations needed for path following
    private Timer actionTimer, opmodeTimer, outtimer; // Timers for progression of states

    private NanoTimer pathTimer;
    double pickup_x = 3.000000;

    double first_pickup_y = 20;

    double turn_distance_x = 20;

    double pickups_y = 32.00;


    private int pathState, armState, outclawState, outgrabState, inclawState, ingrabState; // Different cases and states of the different parts of the robot

    private Pose startPose = new Pose(10, 67.0, Math.toRadians(0)); //TODO

    private Pose readyPose = new Pose(turn_distance_x,pickups_y, Math.toRadians(180)); /// turn spot so make sure it would be safe

    private Pose pickupPose = new Pose(pickup_x, pickups_y, Math.toRadians(180));
    private Pose readyPose1 = new Pose(turn_distance_x,18, Math.toRadians(180)); /// first pickup poses

    private Pose pickupPose1 = new Pose(pickup_x,18,Math.toRadians(180)); /// first pickup poses


    /// hang poses
    private Pose hangPose = new Pose(36.5, 67, Math.toRadians(0)); // TODO runs on

    private Pose firsthangPose = new Pose(36.0,70,0);

    private Pose secondhangPose = new Pose(36.0,68,0);

    private Pose thirdhangPose = new Pose(36.0, 66,0);


    /// push poses  for case transitions
    private Pose pushstart = new  Pose(61,28,Math.toRadians(180)); // has to match

    private Pose firstpushPose = new Pose(20,28, Math.toRadians(0)); // ^^

    private Pose pushstart2 = new Pose(60,18,Math.toRadians(180));

    private Pose endPush = new Pose(20,20, Math.toRadians(0)); /// :D


    private Pose parkPose = new Pose(10,24,0);


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

        pushFirst = follower.pathBuilder()
                .addPath( new BezierCurve(
                        new Point(hangPose),
                        new Point(22.444626743232156, 17.95570139458573, Point.CARTESIAN),
                        new Point(54.81214109926169, 46.54306808859721, Point.CARTESIAN),
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
                .build();

        /// END OF PUSH ALL


        pickup1 = new Path (
                new BezierCurve(
                        new Point(readyPose1),
                        new Point(pickupPose1)
                )
        );
        pickup1.setConstantHeadingInterpolation(Math.toRadians(180));
        first_hang = new Path(
                // Line 3
                new BezierCurve(
                        new Point(pickupPose1),
                        new Point(28.587366694011486, 21.73584905660377, Point.CARTESIAN),
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

        second_hang = new Path(
                // Line 5
                new BezierCurve(
                        new Point(pickupPose),
                        new Point(27.28794093519278, 32.84003281378179, Point.CARTESIAN),
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
                        new Point(27.28794093519278, 30.359310910582437, Point.CARTESIAN),
                        new Point(7.914684167350287, 61.191140278917146, Point.CARTESIAN),
                        new Point(thirdhangPose)
                )
        );
        third_hang.setLinearHeadingInterpolation(pickupPose.getHeading(), Math.toRadians(0));
        /*third_hang_back = new Path(
                // Line 6
                new BezierCurve(
                        new Point(thirdhangPose),
                        new Point(pickupPose)
                )
        );
        third_hang_back.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(320));

         */
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
        park = new Path(
                new BezierCurve(
                        new Point(thirdhangPose),
                        new Point(10,24, Point.CARTESIAN)
                )
        );
        park.setTangentHeadingInterpolation();




    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 2: //go to hang
                follower.followPath(hang_first);

                setPathState(3);
                break; // BREAK

            case 3: //hang
                if (pathTimer.getElapsedTime() > 2*(Math.pow(10,9))){ // TODO: Time to reach hang Position, shorten
                    setPathState(4);
                }
                break; // BREAK
            case 4:
                if (pathTimer.getElapsedTime() > (0.7*(Math.pow(10,9)))) { // TODO : Allowing hang time / release
                    setPathState(5);
                }
                break; // BREAK
            case 5: // Starts the push all curve, don't think we need a wait time here
                follower.followPath(pushFirst);
                setPathState(8);
                break; // BREAK
            case 8:
                if (!follower.isBusy() || follower.getPose().roughlyEquals(firstpushPose, 1)) { // end of push all 3 into the observation zone doesn't stop and continues
                    follower.followPath(pushSecond); // curve forward
                    setPathState(141);
                }
                break; // break

            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(pickup1);
                    setPathState(141);
                }
                break;
            case 141:
                if (!follower.isBusy()) { // TODO pick up time shorten
                    follower.followPath(first_hang);
                    setPathState(145);

                }
                break;
            case 145:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > (2.2*Math.pow(10,9))) { //TODO: HANG CODE time to reach hang pos, then hang shorten
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
                setPathState(156);
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
                    setPathState(161);
                }
                break;
            case 161:
                if(!follower.isBusy()) {
                setPathState(162);
                }
            break;
            case 162:
                if (pathTimer.getElapsedTime() > (1*Math.pow(10,9))) { // TODO time to reach pickup/pickup
                    // pickup
                    follower.followPath(second_hang);
                    setPathState(165);
                }
                break;
            case 165:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > (2.2*Math.pow(10,9))) {// TODO : hang
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
                setPathState(175);
                break;
            case 175:
                if(!follower.isBusy() || pathTimer.getElapsedTime() > (2*Math.pow(10,9))) {
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
                if (pathTimer.getElapsedTime() > (1*Math.pow(10,9))) { // TODO pickup time
                    follower.followPath(third_hang);
                    setPathState(185);
                }
                break;
            case 185:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > (2.2*Math.pow(10,9))) { // wait to reach, hang
                    setPathState(1866);
                }
                break;
            case 1866: // hang
                setPathState(186);

            case 186:
                if (pathTimer.getElapsedTimeSeconds() > 0.7) { // wait hang, for relase
                    setPathState(19);
                }
                break;
            case 19:
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

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
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
