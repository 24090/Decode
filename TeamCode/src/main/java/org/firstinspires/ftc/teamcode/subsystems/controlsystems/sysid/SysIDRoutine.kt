package org.firstinspires.ftc.teamcode.subsystems.controlsystems.sysid

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.internal.files.DataLogger
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.RepeatNTimes
import org.firstinspires.ftc.teamcode.commands.RepeatUntil
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.util.timeSeconds

open class SysIDRoutine<System>(

    val name: String,
    val dataHeaders: List<String>,

    val initSystem: LinearOpMode.() -> System,
    val updateSignal: System.(signal: Double) -> List<Any>,
    val endCondition: System.() -> Boolean,

    val repeatCount: Int = 5,

    val quasistaticSlope: Double = 0.1,
    val dynamicStep: Double = 0.1,
    val dynamicStepRate: Double = 1.0,

): LinearOpMode() {
    override fun runOpMode() {
        val system = initSystem()
        val dataLogger = DataLogger(DataLogger.createFileName("sysId/${name}"))
        dataLogger.addDataLine(listOf("routine", "t", "signal") + dataHeaders)
        val section = { isDynamic: Boolean, isReverse: Boolean ->
            var start = 0.0
            Sequence(
                Instant { telemetry.addLine("Press A to continue"); telemetry.update() },
                WaitUntil { gamepad1.aWasReleased() },
                Instant {
                    start = timeSeconds()
                },
                RepeatUntil(
                    { Instant {
                        val t = timeSeconds() - start
                        val signal = (if (isDynamic) dynamicSignal(t) else quasistaticSignal(t)) * if (isReverse) -1 else 1
                        dataLogger.addDataLine(listOf(t, signal) + system.updateSignal(signal))
                    } },
                    { system.endCondition() }
                ),
                Instant {
                    system.updateSignal(0.0)
                }
            )
        }
        runBlocking(Sequence(
            RepeatNTimes (
                {Sequence(
                    section(true, false),
                    section(true, true),
                )},
                repeatCount
            ),
            RepeatNTimes (
                {Sequence(
                    section(false, false),
                    section(false, true),
                )},
            ),
            Instant { dataLogger.close() }
        ))
    }
    private fun dynamicSignal(t: Double) = (t/dynamicStepRate).toInt() * dynamicStep
    private fun quasistaticSignal(t: Double) = t * quasistaticSlope
}