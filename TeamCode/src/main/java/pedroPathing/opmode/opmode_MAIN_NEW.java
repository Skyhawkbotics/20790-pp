package pedroPathing.opmode;

import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


/**
 * This is the TeleOpEnhancements OpMode. It is an example usage of the TeleOp enhancements that
 * Pedro Pathing is capable of.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/21/2024
 *///sadfasdfilkajslkdfjaslk
@TeleOp(name = "opmode_MAIN_NEW", group = "MAIN")
public class opmode_MAIN_NEW extends OpMode {
    private Follower follower;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    //define these up here instead of in the init section like rr, idk why but it seems to work fine.
    private Servo servo_outtake_flip1, servo_outtake_flip2, servo_intake_wrist, servo_intake_rotate, sweeper, servo_intake;
    private CRServo servo_outtake;
    private DcMotorEx up1, up2, out;
    private TouchSensor up_zero, out_zero;
    //from rr version of opmode_MAIN
    int arm_upper_lim = 4000;
    int up_true_target_pos;
    int out_true_target_pos;
    double servo_outtake_flip_location = 0;
    double servo_intake_wrist_location = 0;
    double servo_intake_rotate_location = 0.47;
    int out_max_pos = 1330;

    int up_specimen_hang = 1907; // Viper

    double outtake_specimen_hang = 0.45;

    double driving_multiplier_fast = 0.7;
    double driving_multiplier_slow = 0.3;

    double driving_multiplier;

    private final ElapsedTime runtime = new ElapsedTime();


    //path
    private PathChain park;

    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void start() {
        //PATHING
    }

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.startTeleopDrive();


        //from rr version

        //setup arm to use velocity
        //setup arm variable
        up1 = hardwareMap.get(DcMotorEx.class, "up1");
        up1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up1.setDirection(DcMotorSimple.Direction.REVERSE);

        up2 = hardwareMap.get(DcMotorEx.class, "up2");
        up2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Pose startPose = new Pose(15, 40.0, Math.toRadians(0)); //TODO
        follower.setStartingPose(startPose);
    }

    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    @Override
    public void loop() {
        //drive code from TeleOpEnhancements


        //TESTING PATH THING VERSION

        //change drive speed for more accuracy if needed
        if (gamepad1.left_bumper) {
            driving_multiplier = driving_multiplier_slow;
        } else {
            driving_multiplier = driving_multiplier_fast;
        }

        //BUMPERS STRAFE
        if (gamepad1.left_bumper) {
            follower.setTeleOpMovementVectors(0, 1, 0); //TODO: SWITCH IF NECESARRY
        } else if (gamepad1.right_bumper) {
            follower.setTeleOpMovementVectors(0, -1, 0); //TODO: SWITCH IF NECESARRY
        } else {
            follower.setTeleOpMovementVectors(0, 0, 0);

        }


        viper_slide();

        //SWEEPER:

        //telemetry
        telemetry.addData("gamepad2.rightstickx", gamepad2.right_stick_x);
        telemetry.addData("gamepad2.rightsticky", gamepad2.right_stick_y);
        telemetry.addData("up1 pos", up1.getCurrentPosition());
        telemetry.addData("up2 pos", up2.getCurrentPosition());
        telemetry.update();
    }


    public void viper_slide() {
        if (gamepad2.triangle) { // Up
            up1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("up1", true);
            up1.setVelocity(40);
            up_true_target_pos = 0;
        } else if (gamepad2.x) { // Down
            up1.setVelocity(-40);
            telemetry.addData("down1", true);
            up_true_target_pos = 0;
        } else if (gamepad2.dpad_up) {
            up2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            up2.setVelocity(40);
            telemetry.addData("up2", true);
            up_true_target_pos = 0;
        } else if (gamepad2.dpad_down) {
            up2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            up2.setVelocity(-40);
            telemetry.addData("down2", true);
            up_true_target_pos = 0;

        } else {
            up1.setPower(0.5);
            up2.setPower(0.5);
            if (up_true_target_pos == 0) {
                up1.setTargetPosition(up1.getCurrentPosition());
                up2.setTargetPosition(up2.getCurrentPosition());
                up_true_target_pos = up1.getCurrentPosition();
            }
            up1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            up2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}