# Twilio Call Test Script

A minimal script to test making a phone call using Twilio.

## Setup

1. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Get your Twilio credentials:**
   - Go to [Twilio Console](https://console.twilio.com/)
   - Copy your Account SID and Auth Token
   - Get a Twilio phone number (or use a trial number)

3. **Create your `.env` file:**
   - Copy `env_template.txt` to `.env`:
     ```bash
     cp env_template.txt .env
     ```
   - Edit `.env` and fill in your actual credentials:
     ```
     TWILIO_ACCOUNT_SID=ACxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
     TWILIO_AUTH_TOKEN=your_auth_token_here
     TWILIO_PHONE_NUMBER=+1234567890
     ```

4. **Run the test script:**
   ```bash
   python test_call.py
   ```

## Notes

- The script calls the number **647 519 4566**
- It uses Twilio's demo TwiML URL for the call content
- Make sure your Twilio account has calling enabled (trial accounts have limitations)

