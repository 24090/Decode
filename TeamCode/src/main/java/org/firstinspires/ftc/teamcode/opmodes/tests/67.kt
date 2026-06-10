package org.firstinspires.ftc.teamcode.opmodes.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.ForeverCommand
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherWait

@TeleOp(name="67")
class Flapper67: LinearOpMode() {
    override fun runOpMode() {
        val intake = Intake(hardwareMap)
        waitForStart()
        runBlocking(Sequence(
            intake.releaseLeft(),
            Sleep(1.0),
            intake.releaseRight(),
            Sleep(1.0),


            intake.releaseLeft(),
            Sleep(1.0),
            intake.releaseRight(),
            Sleep(1.0),


            intake.releaseLeft(),
            Sleep(1.0),
            intake.releaseRight(),
            Sleep(1.0),
        ))
    }

}