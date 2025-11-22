import os
from dotenv import load_dotenv
from twilio.rest import Client

# Load environment variables from .env file
load_dotenv()

# Get Twilio credentials from environment variables
account_sid = os.getenv('TWILIO_ACCOUNT_SID')
auth_token = os.getenv('TWILIO_AUTH_TOKEN')
twilio_phone = os.getenv('TWILIO_PHONE_NUMBER')

# Validate that all required variables are set
if not all([account_sid, auth_token, twilio_phone]):
    raise ValueError("Missing required environment variables. Please check your .env file.")

# Check for placeholder values
if 'your_account_sid_here' in account_sid or account_sid.startswith('your_'):
    raise ValueError("ERROR: Your .env file still contains placeholder values!\n"
                     "Please update TWILIO_ACCOUNT_SID with your actual Twilio Account SID from https://console.twilio.com/")
if 'your_auth_token_here' in auth_token or auth_token.startswith('your_'):
    raise ValueError("ERROR: Your .env file still contains placeholder values!\n"
                     "Please update TWILIO_AUTH_TOKEN with your actual Twilio Auth Token from https://console.twilio.com/")
if twilio_phone == '+1234567890' or not twilio_phone.startswith('+'):
    raise ValueError("ERROR: Your .env file still contains placeholder values!\n"
                     "Please update TWILIO_PHONE_NUMBER with your actual Twilio phone number (format: +1234567890)")

# Initialize Twilio client
client = Client(account_sid, auth_token)

# Phone number to call
to_number = "+16475194566"  # 647 519 4566

print(f"Calling {to_number} from {twilio_phone}...")

# Make the call
call = client.calls.create(
    to=to_number,
    from_=twilio_phone,
    url="http://demo.twilio.com/docs/voice.xml"  # Twilio demo TwiML
)

print(f"Call initiated! Call SID: {call.sid}")
print(f"Call status: {call.status}")

