package org.firstinspires.ftc.teamcode.commands

fun RepeatNTimes(command: () -> Command, n: Int = 1): RepeatUntil {
    var n = n
    return RepeatUntil({n -= 1; command()},{n > 0})
}