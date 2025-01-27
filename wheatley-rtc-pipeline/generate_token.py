# server.py
import os
from livekit import api

from dotenv import load_dotenv

load_dotenv(dotenv_path=".env.local")

def getToken():
  token = api.AccessToken(os.getenv('LIVEKIT_API_KEY'), os.getenv('LIVEKIT_API_SECRET')) \
    .with_identity("identity") \
    .with_name("my name") \
    .with_grants(api.VideoGrants(
        room_join=True,
        room="my-room",
    ))
  return token.to_jwt()

print(getToken())