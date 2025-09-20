package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Parallel
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.drivetrain.Drive
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.util.BulkReads

@TeleOp(name="Controlled")
class Controlled: LinearOpMode() {
    override fun runOpMode() {
        val bulkReads = BulkReads(hardwareMap)
        val drive = Drive(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val intake = Intake(hardwareMap)
        val driveFunction = {
            drive.strafe = gamepad1.left_stick_x.toDouble();
            drive.drive = -gamepad1.left_stick_y.toDouble();
            drive.turn = gamepad1.right_stick_x.toDouble();
            drive.setMotorPowers()
        }
        waitForStart()
        runBlocking(shooter.spinUp())
        while (opModeIsActive()){
            bulkReads.update()
            driveFunction()
            shooter.update()
            intake.update()
            if (gamepad1.aWasReleased()) {
                runBlocking(intake.spinUp())
            }
            if (gamepad1.bWasReleased()) {
                runBlocking(intake.spinDown())
            }
            if (gamepad1.xWasReleased()) {
                runBlocking(Race(
                    Forever {
                        bulkReads.update()
                        driveFunction()
                        shooter.update()
                        intake.update()
                    },
                    intake.releaseBall()
                ))
            }
        }
    }

}