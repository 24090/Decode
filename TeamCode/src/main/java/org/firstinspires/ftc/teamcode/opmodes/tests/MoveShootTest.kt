package org.firstinspires.ftc.teamcode.opmodes.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.opmodes.poses.scorePosition
import org.firstinspires.ftc.teamcode.opmodes.poses.startPose
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.PDLT
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.hD
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.hP
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.hT
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kA
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kS
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kV
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.tipAccelBackward
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.tipAccelForward
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.getRelativeVelocity
import org.firstinspires.ftc.teamcode.subsystems.drive.processTurnTranslational
import org.firstinspires.ftc.teamcode.subsystems.drive.tipCorrected
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherLeftBack
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherLeftForward
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherRightBack
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherRightForward
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.reads.VoltageReader.controlHubVoltage
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.util.calculatePredictiveMoveShoot

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
        drive.follow = {
            val outputs = outputs
            val translational = Vector.fromCartesian(-gamepad1.left_stick_x.toDouble(), gamepad1.left_stick_y.toDouble()).rotated(-drive.localizer.heading)

            val turn = if (outputs?.second == null){
                0.0
            } else {
                PDLT(AngleUnit.normalizeRadians(outputs.second - drive.localizer.heading), -drive.localizer.headingVel, hP, hD, kS, hT)
            }

            processTurnTranslational(turn, translational, drive.localizer.pose, drive.localizer.poseVel)
        }
        runBlocking(
            Forever {
                reads.update()
                outputs = calculatePredictiveMoveShoot(0.0,drive.localizer.pose, drive.localizer.poseVel)
                recordTime("reads")
                drive.update()
                val relativePose = (scorePosition - Vector.fromPose(drive.localizer.pose))
                telemetry.addData("target exit velocity", outputs?.first)
                telemetry.addData("target heading", outputs?.second)
                telemetry.addData("pose", relativePose)
                telemetry.addData("pose", drive.localizer.poseVel)
                telemetry.addData("motorLeftVelocity", shooter.motorLeft.velocity)
                telemetry.addData("motorRightVelocity", shooter.motorRight.velocity)
                intake.pusherLeft.position = if (gamepad1.x) pusherLeftForward else pusherLeftBack
                intake.pusherRight.position = if (gamepad1.x) pusherRightForward else pusherRightBack
                shooter.update(); recordTime("shooter")
                intake.update(); recordTime("intake")
                telemetry.update()
            }
        )
    }
}