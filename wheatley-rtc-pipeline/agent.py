import logging

from dotenv import load_dotenv
from livekit.agents import (
    AutoSubscribe,
    JobContext,
    JobProcess,
    WorkerOptions,
    cli,
    llm,
)
from livekit.agents.pipeline import VoicePipelineAgent
from livekit.plugins import openai, silero, elevenlabs


load_dotenv(dotenv_path=".env.local")
logger = logging.getLogger("voice-agent")

eleven_tts = elevenlabs.tts.TTS(
    model="eleven_multilingual_v2",
    voice=elevenlabs.tts.Voice(
        id="9DiCz8o7648uuGkrlfzc",
        name="Wheatley",
        category="voice design",
        settings=elevenlabs.tts.VoiceSettings(
            stability=0.29,
            similarity_boost=0.67,
            style=0.41,
            use_speaker_boost=True
        ),
    ),
    streaming_latency=3,
    enable_ssml_parsing=False,
    chunk_length_schedule=[80, 120, 200, 260],
)


def prewarm(proc: JobProcess):
    proc.userdata["vad"] = silero.VAD.load()


async def entrypoint(ctx: JobContext):
    initial_ctx = llm.ChatContext().append(
        role="system",
        text=(
            "You are Wheatley, the humorous and slightly inept AI from Portal. Your responses should be witty, self-deprecating, and full of personality. "
            "Stick to short, quirky remarks, and maintain a conversational, slightly scatterbrained tone. "
            "Avoid technical jargon unless humorously misinterpreted. Your charm lies in your comedic timing and endearing clumsiness."
        )
    )

    logger.info(f"connecting to room {ctx.room.name}")
    await ctx.connect(auto_subscribe=AutoSubscribe.AUDIO_ONLY)

    # Wait for the first participant to connect
    participant = await ctx.wait_for_participant()
    logger.info(f"starting voice assistant for participant {participant.identity}")

    # This project is configured to use Deepgram STT, OpenAI LLM and TTS plugins
    # Other great providers exist like Cartesia and ElevenLabs
    # Learn more and pick the best one for your app:
    # https://docs.livekit.io/agents/plugins
    agent = VoicePipelineAgent(
        vad=ctx.proc.userdata["vad"],
        stt=openai.STT(),
        llm=openai.LLM(model="gpt-4o-mini"),
        tts=eleven_tts,
        chat_ctx=initial_ctx,
    )

    agent.start(ctx.room, participant)

    # The agent should be polite and greet the user when it joins :)
    await agent.say("Hey, how can I help you today?", allow_interruptions=True)


if __name__ == "__main__":
    cli.run_app(
        WorkerOptions(
            entrypoint_fnc=entrypoint,
            prewarm_fnc=prewarm,
        ),
    )
