package pedroPathing.autonomous.unsured;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
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
@Autonomous(name = "arm auto arm", group = "AUTO")
public class arm_auto_code extends OpMode {
    private Follower follower;
    private Timer opmodeTimer; // Timers for progression of states
    private NanoTimer pathTimer;
    private int pathState, armState, outclawState, outgrabState, inclawState, ingrabState, sweeperState; // Different cases and states of the different parts of the robot
    private Pose startPose = new Pose(10, 67.0, Math.toRadians(0)); //TODO
    private DcMotorEx up, out;

    private Servo servo_outtake_wrist, servo_intake_wrist, sweeper;
    private CRServo servo_outtake, servo_intake;
    private TouchSensor up_zero;
    private Telemetry telemetryA;
    int up_hanging_position = 1755; //DONE: calibrate this value, viper slide position to
    int up_hanging_position_done = 1560; //TODO: calibrate this value, position of viper slide when releasing after speciman is on the bar.
    int preload_hang_pos = 2160;
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setSweeperState(-1);
                setPathState(1);
                break;
            case 1:
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setSweeperState(3);
                    setPathState(2);
                }
                break;
            case 2:
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setSweeperState(1);
                    setPathState(3);
                }
                break;
            case 3:
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setSweeperState(2);
                }
                break;


        }
    }
    public void autonomousActionUpdate() {
        switch (armState) {
            case -1:
                up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case 0:
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
        switch (sweeperState) {
            case -1:
                sweeper.setPosition(1);
                break;
            case 1: //release
                sweeper.setPosition(0.14);
                break;
            case 2:
                sweeper.setPosition(0.20);
                break;
            case 3:
                sweeper.setPosition(0.4);
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
    public void setSweeperState(int mstate) {
        sweeperState = mstate;
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
    @Override
    public void init() {
        pathTimer = new NanoTimer();
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
        int charles = out.getCurrentPosition();
        out.setTargetPosition(charles);
        out.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        out.setPower(1);
        servo_outtake = hardwareMap.get(CRServo.class,"outtake");

        sweeper = hardwareMap.get(Servo.class, "sweeper");


        servo_intake = hardwareMap.get(CRServo.class, "intake");



        servo_intake_wrist = hardwareMap.get(Servo.class, "intakeWrist");

        servo_outtake_wrist = hardwareMap.get(Servo.class, "outtakeWrist");
        servo_outtake_wrist.setPosition(0);


        up_zero = hardwareMap.get(TouchSensor.class, "up_zero");


    }
    @Override
    public void init_loop() {


        if (opmodeTimer.getElapsedTimeSeconds() > 4) {
            telemetryA.addData("Init", "Finished");
        }
    }
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        setArmState(-1);
        setoutGrabState(0);
        setinclawState(0);
        setIngrabState(0);
        setoutClawState(0);


    }
    @Override
    public void stop() {
    }
}
