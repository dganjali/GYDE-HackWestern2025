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


def build_alert_message(event_type: str,
                        resident_name: str = "your loved one",
                        location: str = "home") -> str:
    """
    Build a spoken alert message based on event type and context.
    """
    event_type = (event_type or "").upper()

    if event_type == "FALL":
        return (
            f"Alert. A possible fall has been detected for {resident_name} "
            f"at {location}. Please check on them immediately."
        )
    elif event_type == "NO_MOVEMENT":
        return (
            f"Alert. {resident_name} has shown no movement for a prolonged period "
            f"at {location}. Please check on them."
        )
    elif event_type == "LOW_BATTERY":
        return (
            f"This is a system notice. The companion robot battery is low "
            f"at {location}. Please remind {resident_name} to charge it."
        )
    else:
        return (
            f"Alert. An event has been detected for {resident_name} at {location}."
        )


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


def trigger_call_for_event(
    to_number: str,
    event_type: str,
    resident_name: str = "your loved one",
    location: str = "home",
) -> str:
    """
    High-level helper: build a contextual alert message for the given event
    and place an outbound call to `to_number`.

    Returns the Twilio call SID.
    """
    message = build_alert_message(event_type, resident_name, location)
    print(f"[Alert] Event={event_type}, resident={resident_name}, location={location}")
    return trigger_call(to_number, message)


if __name__ == "__main__":
    # Manual test for dynamic event-based call
    test_number = os.getenv("TEST_NUMBER")
    if not test_number:
        print("Set TEST_NUMBER env var for testing.")
    else:
        # Example: simulate a FALL event for 'Grandma Li' in 'the living room'
        trigger_call_for_event(
            to_number=test_number,
            event_type="FALL",
            resident_name="Grandma Li",
            location="the living room"
        )

