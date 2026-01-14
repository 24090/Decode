package org.firstinspires.ftc.teamcode.opmodes.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter

@TeleOp()
class EncoderTest: LinearOpMode(){
    override fun runOpMode() {
        val reads = Reads(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val intake = Intake(hardwareMap)
        waitForStart()
        runBlocking(
            Race(
                Forever {
                    reads.update()
                    telemetry.addData("left shooter", shooter.motorLeft.velocity)
                    telemetry.addData("right shooter", shooter.motorRight.velocity)
                    telemetry.addData("intake", intake.motor.velocity)
                    telemetry.addData("back intake", intake.motorBack.velocity)
                    telemetry.update()
                }
            )
        )
    }
}