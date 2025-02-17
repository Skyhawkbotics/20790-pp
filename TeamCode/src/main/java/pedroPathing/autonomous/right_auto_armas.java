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
@Autonomous(name = "right auto arm", group = "AUTO")
// 18.5 inches away from observation zone
public class right_auto_armas extends OpMode {

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


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setArmState(1);
                setPathState(1);
                break;
            case 1:
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setArmState(3);
                    setPathState(2);
                }
                break;
            case 2:
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setArmState(0);
                    setPathState(3);
                }
                break;
            case 3:
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setArmState(2);
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
                break;
            case 2:
                up.setTargetPosition(150);
                up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                up.setPower(0.7);
                break;

            case 3:
                up.setTargetPosition(1560);
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
        opmodeTimer.resetTimer();
        setPathState(0);
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
