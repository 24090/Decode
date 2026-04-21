package org.firstinspires.ftc.teamcode.commands

fun RepeatNTimes(command: (n: Int) -> Command, N: Int = 0): RepeatCommandUntil {
    var n = 0
    return RepeatCommandUntil({n += 1; command(n-1)},{n < N})
}