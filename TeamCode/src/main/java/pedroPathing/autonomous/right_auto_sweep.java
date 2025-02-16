package pedroPathing.autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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


    private int pathState, armState, outclawState, outgrabState, inclawState, ingrabState; // Different cases and states of the different parts of the robot
    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Centerstage, this would be blue far side/red human player station.)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y. **/
    //Start Pose
    private Pose startPose = new Pose(10, 67.0, Math.toRadians(0)); //TODO

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
    private PathChain hang1;

    private Path one, two, three, four, five, six, seven, eight, nine, ten, eleven, twelve, thirteen, fourteen, fifteen, sixteen, seventeen, eighteen;

    // Motors
    private DcMotorEx up, out;
    private Servo servo_outtake_wrist, servo_intake_wrist, servo_intake_rotate;
    private CRServo servo_outtake, servo_intake;
    private TouchSensor up_zero, out_zero;
    private Telemetry telemetryA;
    double intake_wrist_pos_transfer = 0;
    double outtake_wrist_pos_transfer = 0;
    int up_hanging_position = 1775; //DONE: calibrate this value, viper slide position to
    int up_hanging_position_done = 1270; //TODO: calibrate this value, position of viper slide when releasing after speciman is on the bar.
    // 1543
    //0.29

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        one = new Path(
                new BezierLine(
                        new Point(8.000, 64.000, Point.CARTESIAN),
                        new Point(36.500, 64.000, Point.CARTESIAN)
                )
        );
        one.setConstantHeadingInterpolation(0);
        two = new Path(
                new BezierCurve(
                        new Point(36.500, 64.000, Point.CARTESIAN),
                        new Point(15.000, 37.500, Point.CARTESIAN),
                        new Point(35.000, 35.000, Point.CARTESIAN)
                )
        );
        two.setLinearHeadingInterpolation(0, Math.toRadians(300));
        three = new Path(
                new BezierLine(
                        new Point(35.000, 35.000, Point.CARTESIAN),
                        new Point(30.000, 35.000, Point.CARTESIAN)
                )
        );
        three.setLinearHeadingInterpolation(Math.toRadians(300), Math.toRadians(200));
        four = new Path(
                new BezierLine(
                        new Point(30.000, 35.000, Point.CARTESIAN),
                        new Point(35.000, 29.000, Point.CARTESIAN)
                )
        );
        four.setConstantHeadingInterpolation(300);
        five = new Path(
                new BezierLine(
                        new Point(35.000, 29.000, Point.CARTESIAN),
                        new Point(30.000, 29.000, Point.CARTESIAN)
                )
        );
        five.setLinearHeadingInterpolation(Math.toRadians(300), Math.toRadians(200));
        six = new Path(
                new BezierLine(
                        new Point(30.000, 29.000, Point.CARTESIAN),
                        new Point(35.000, 24.000, Point.CARTESIAN)
                )
        );
        six.setConstantHeadingInterpolation(300);
        seven = new Path(
                new BezierLine(
                        new Point(35.000, 24.000, Point.CARTESIAN),
                        new Point(30.000, 24.000, Point.CARTESIAN)
                )
        );
        seven.setLinearHeadingInterpolation(Math.toRadians(300), Math.toRadians(180));
        eight = new Path(
                new BezierLine(
                        new Point(30.000, 24.000, Point.CARTESIAN),
                        new Point(6.000, 24.000, Point.CARTESIAN)
                )
        );
        eight.setConstantHeadingInterpolation(Math.toRadians(180));
        nine = new Path(
                new BezierCurve(
                        new Point(6.000, 24.000, Point.CARTESIAN),
                        new Point(37.000, 21.500, Point.CARTESIAN),
                        new Point(10.000, 55.000, Point.CARTESIAN),
                        new Point(36.000, 70.000, Point.CARTESIAN)
                )
        );
        nine.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0));
        ten = new Path(
                new BezierCurve(
                        new Point(36.000, 70.000, Point.CARTESIAN),
                        new Point(20.000, 59.000, Point.CARTESIAN),
                        new Point(20.000, 37.000, Point.CARTESIAN)
                )
        );
        ten.setLinearHeadingInterpolation(0, Math.toRadians(180));
        eleven = new Path(
                new BezierLine(
                        new Point(20.000, 37.000, Point.CARTESIAN),
                        new Point(6.000, 37.000, Point.CARTESIAN)
                )
        );
        eleven.setConstantHeadingInterpolation(Math.toRadians(180));
        twelve = new Path(
                new BezierCurve(
                        new Point(6.000, 37.000, Point.CARTESIAN),
                        new Point(37.000, 39.000, Point.CARTESIAN),
                        new Point(7.000, 55.000, Point.CARTESIAN),
                        new Point(36.000, 68.000, Point.CARTESIAN)
                )
        );
        twelve.setLinearHeadingInterpolation(Math.toRadians(180), 0);
        thirteen = new Path(
                new BezierCurve(
                        new Point(36.000, 68.000, Point.CARTESIAN),
                        new Point(20.000, 59.000, Point.CARTESIAN),
                        new Point(20.000, 37.000, Point.CARTESIAN)
                )
        );
        thirteen.setLinearHeadingInterpolation(0, Math.toRadians(180));
        thirteen.setReversed(true);
        fourteen = new Path(
                new BezierLine(
                        new Point(20.000, 37.000, Point.CARTESIAN),
                        new Point(6.000, 37.000, Point.CARTESIAN)
                )
        );
        fourteen.setConstantHeadingInterpolation(Math.toRadians(180));
        fifteen = new Path(
                new BezierCurve(
                        new Point(6.000, 37.000, Point.CARTESIAN),
                        new Point(37.000, 39.000, Point.CARTESIAN),
                        new Point(7.000, 55.000, Point.CARTESIAN),
                        new Point(36.000, 66.000, Point.CARTESIAN)
                )
        );
        fifteen.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0));
        sixteen = new Path(
                new BezierCurve(
                        new Point(36.000, 66.000, Point.CARTESIAN),
                        new Point(20.000, 59.000, Point.CARTESIAN),
                        new Point(20.000, 37.000, Point.CARTESIAN)
                )
        );
        sixteen.setLinearHeadingInterpolation(0, Math.toRadians(180));
        seventeen = new Path(
                new BezierLine(
                        new Point(20.000, 37.000, Point.CARTESIAN),
                        new Point(6.000, 37.000, Point.CARTESIAN)
                )
        );
        seventeen.setConstantHeadingInterpolation(Math.toRadians(180));
        eighteen = new Path(
                new BezierCurve(
                        new Point(6.000, 37.000, Point.CARTESIAN),
                        new Point(37.000, 39.000, Point.CARTESIAN),
                        new Point(7.000, 55.000, Point.CARTESIAN),
                        new Point(36.000, 62.000, Point.CARTESIAN)
                )
        );
        eighteen.setLinearHeadingInterpolation(0, Math.toRadians(180));

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() function on line 193)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. **/
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(one);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(two);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(three);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(four);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(five);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(six);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(seven);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(eight);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(nine);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(ten);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(eleven);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(twelve);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(thirteen);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(fourteen);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(fifteen);
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(sixteen);
                    setPathState(17);
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    follower.followPath(seventeen);
                    setPathState(18);
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    follower.followPath(eighteen);
                    setPathState(-1);
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
                servo_outtake_wrist.setPosition(0.25);
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
