package org.firstinspires.ftc.teamcode.commands

class Scheduler(name: String = "Scheduler"): OverrideButtonCommand(name, true){
    val commands = ArrayList<Command>()

    fun schedule(command: Command){
        commands.add(command)
    }

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
        for (i in 0..commands.size-1){
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
        return CommandResult.Continue
    }
}