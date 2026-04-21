package org.firstinspires.ftc.teamcode.subsystems.controlsystems.sysid

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.internal.files.DataLogger
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.util.timeSeconds

open class SysIDRoutine<System>(

    val name: String,

    val dataHeaders: Array<String>,

    val initSystem: LinearOpMode.() -> System,
    val updateSignal: System.(signal: Double) -> Array<Any>,
    val endCondition: System.(signal: Double, t: Double) -> Boolean,

    val repeatCount: Int = 5,

    val quasistaticSlope: Double = 0.1,
    val dynamicStep: Double = 0.1,
    val dynamicStepRate: Double = 1.0,
): LinearOpMode() {
    override fun runOpMode() {
        val system = initSystem()
        val dataLogger = DataLogger("/data/sysid/${DataLogger.createFileName(name)}")
        dataLogger.addDataLine("routine", "t", "signal", *dataHeaders)
        waitForStart()
        val section = { testType: TestType, n: Int ->
            var start = 0.0
            var t = 0.0
            var signal = 0.0
            Sequence(
                Instant { telemetry.addLine("Press A to continue"); telemetry.update() },
                Sleep ( 1.0 ),
                Instant { start = timeSeconds() },
                WaitUntil {
                    t = timeSeconds() - start
                    signal = getSignal(testType, t)
                    telemetry.addLine("signal: ${signal}")
                    telemetry.addLine("${testType.name} ${n}")
                    telemetry.update()

                    dataLogger.addDataLine(
                        "${testType.name}-${n}",
                        t,
                        signal,
                        *system.updateSignal(signal)
                    )
                    return@WaitUntil system.endCondition(signal, t)
                },
                Instant {
                    system.updateSignal(0.0)
                }
            )
        }
        runBlocking(Sequence(
                section(TestType.DynamicForward, 0),
                section(TestType.DynamicReverse, 0),
                section(TestType.QuasistaticForward, 0),
                section(TestType.QuasistaticReverse, 0),
                Instant { dataLogger.close() }
        ))
    }
    fun getSignal(testType: TestType, t: Double) = when(testType) {
        TestType.DynamicReverse -> -dynamicSignal(t)
        TestType.DynamicForward -> dynamicSignal(t)
        TestType.QuasistaticReverse -> -quasistaticSignal(t)
        TestType.QuasistaticForward -> quasistaticSignal(t)
    }
    private fun dynamicSignal(t: Double) = (t/dynamicStepRate).toInt() * dynamicStep
    private fun quasistaticSignal(t: Double) = t * quasistaticSlope

    enum class TestType() {
        DynamicForward,
        DynamicReverse,
        QuasistaticForward,
        QuasistaticReverse,
    }
}