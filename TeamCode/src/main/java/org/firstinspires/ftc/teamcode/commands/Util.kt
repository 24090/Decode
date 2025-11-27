package org.firstinspires.ftc.teamcode.commands

fun Sleep(timeSeconds: Double, name: String = "Sleep") = Future({
        val targetTime = System.currentTimeMillis() + timeSeconds * 1000
        WaitUntil({System.currentTimeMillis() >= targetTime}, "SleepSubcommand")
    }, name)