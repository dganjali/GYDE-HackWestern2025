import os
from dotenv import load_dotenv
from twilio.rest import Client

# Load environment variables from .env file
load_dotenv()

# Load environment variables (support both TWILIO_PHONE_NUMBER and TWILIO_FROM_NUMBER)
TWILIO_ACCOUNT_SID = os.getenv("TWILIO_ACCOUNT_SID")
TWILIO_AUTH_TOKEN = os.getenv("TWILIO_AUTH_TOKEN")
TWILIO_FROM_NUMBER = os.getenv("TWILIO_FROM_NUMBER") or os.getenv("TWILIO_PHONE_NUMBER")

# Validate required variables
if not all([TWILIO_ACCOUNT_SID, TWILIO_AUTH_TOKEN, TWILIO_FROM_NUMBER]):
    raise ValueError("Missing required environment variables. Please check your .env file.")

# Initialize Twilio client
client = Client(TWILIO_ACCOUNT_SID, TWILIO_AUTH_TOKEN)


def trigger_call(to_number: str, message: str) -> str:
    """
    Places an outbound call to `to_number` and uses inline TwiML <Say>
    to speak `message`.

    Returns the call SID.
    """
    twiml = f"<Response><Say>{message}</Say></Response>"

    call = client.calls.create(
        to=to_number,
        from_=TWILIO_FROM_NUMBER,
        twiml=twiml
    )

    print(f"[Twilio] Outbound call placed. SID={call.sid}")
    return call.sid


if __name__ == "__main__":
    # Check if trigger variable is set to 1
    trigger = os.getenv("TRIGGER_CALL", "0")
    
    if trigger == "1":
        test_number = os.getenv("TEST_NUMBER")
        if not test_number:
            print("ERROR: TRIGGER_CALL=1 but TEST_NUMBER is not set in .env file.")
        else:
            print(f"[Trigger] TRIGGER_CALL=1 detected. Calling {test_number}...")
            trigger_call(
                test_number,
                "Fall detected at home. Please check immediately."
            )
    else:
        print(f"TRIGGER_CALL is set to '{trigger}'. Set TRIGGER_CALL=1 in .env to execute call.")

