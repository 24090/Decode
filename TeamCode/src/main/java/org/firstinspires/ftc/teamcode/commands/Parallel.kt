package org.firstinspires.ftc.teamcode.commands

class Parallel(vararg commands: Command): OverrideButtonCommand("Sequence", true){
    val commands = ArrayList(commands.map {c -> c})
    override fun getFuture(): FutureCommand{
        return FutureCommand(commands.map(Command::getFuture), name, selfCondense, uid)
    }

    override fun getDead(reason: CommandResult.End): DeadCommand{
        return DeadCommand(
            reason,
            commands.map{ c ->
                c.getDead(CommandResult.End(Result.failure(Error("Parent ended early"))))
            },
            name,
            selfCondense,
            uid
        )
    }

    override fun run(): CommandResult {
        var alldead = true
        for (i in 0..commands.size-1){
            alldead = alldead && (commands[i] is DeadCommand)
            val result = commands[i].update()
            when(result){
                CommandResult.Continue -> continue
                is CommandResult.End -> {
                    if (result.result.isFailure){
                        return CommandResult.End(Result.failure(Error("Child error")))
                    }
                    commands[i] = commands[i].getDead(result)
                }
            }
        }
        return if (alldead) {
            CommandResult.End(Result.success("No Commands Left"))
        } else {
            CommandResult.Continue
        }
    }
}