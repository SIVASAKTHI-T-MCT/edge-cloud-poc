import os, time
from dotenv import load_dotenv
from openai import OpenAI

load_dotenv()
client = OpenAI(
    base_url="https://api.tokenfactory.nebius.com/v1/",
    api_key=os.environ["NEBIUS_API_KEY"],
)

def run(label, **extra):
    t0 = time.time()
    r = client.chat.completions.create(
        model=os.environ["NEBIUS_MODEL"],
        messages=[{"role": "user", "content": "Say hello in one short sentence."}],
        max_tokens=100,
        **extra,
    )
    dt = time.time() - t0
    print(f"{label}: {dt:.2f} s | completion tokens: {r.usage.completion_tokens} | reply: {r.choices[0].message.content!r}")

run("Thinking default")
run("Thinking off", extra_body={"chat_template_kwargs": {"enable_thinking": False}})