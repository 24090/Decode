package org.firstinspires.ftc.teamcode.opmodes.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.opmodes.commands.releasePattern
import org.firstinspires.ftc.teamcode.opmodes.poses.startPose
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.Vector
import org.firstinspires.ftc.teamcode.subsystems.huskylens.HuskyLens
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherLeftBack
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherLeftForward
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherRightBack
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherRightForward
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.util.IndexTracker
import org.firstinspires.ftc.teamcode.util.Pattern
import org.firstinspires.ftc.teamcode.util.calculatePredictiveMoveShoot
import org.firstinspires.ftc.teamcode.util.moveShootKinematics
import kotlin.math.sqrt

@TeleOp
class MoveShootTest: LinearOpMode(){
    override fun runOpMode() {
        val reads = Reads(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val intake = Intake(hardwareMap)
        val drive = Drive(hardwareMap)
        reads.update()
        var time = System.currentTimeMillis()
        val recordTime = { name:String ->
            val newTime = System.currentTimeMillis()
            telemetry.addData("$name (ms)", newTime - time)
            time = newTime
        }
        waitForStart()
        drive.localizer.pose = startPose
        var outputs: Pair<Double, Double>? = null
        drive.currentUpdateTranslational = {
            val v = Vector.fromCartesian(-gamepad1.left_stick_x.toDouble(), gamepad1.left_stick_y.toDouble())
            drive.strafe = v.rotated(-drive.localizer.heading).y
            drive.drive =  v.rotated(-drive.localizer.heading).x
        }
        drive.currentUpdateHeading = {
            drive.targetPose.heading = outputs?.second ?: drive.targetPose.heading
            drive.updateHeading()
        }
        runBlocking(
            Forever {
                reads.update()
                outputs = calculatePredictiveMoveShoot(drive.localizer.pose, drive.localizer.poseVel)
                recordTime("reads")
                drive.update()
                telemetry.addData("target exit velocity", outputs?.first)
                telemetry.addData("target heading", outputs?.second)
                telemetry.addData("pose", drive.localizer.pose)
                telemetry.addData("pose", drive.localizer.poseVel)
                telemetry.addData("targetVelocity", shooter.targetVelocity)
                telemetry.addData("motorLeftVelocity", shooter.motorLeft.velocity)
                telemetry.addData("motorRightVelocity", shooter.motorRight.velocity)
                intake.pusherLeft.position = if (gamepad1.x) pusherLeftForward else pusherLeftBack
                intake.pusherRight.position = if (gamepad1.x) pusherRightForward else pusherRightBack
                shooter.targetVelocity = shooter.exitVelocityToVelocityLUT.get(outputs?.first ?: 1400.0)
                shooter.update(); recordTime("shooter")
                intake.update(); recordTime("intake")
                telemetry.update()
            }
        )
    }
}