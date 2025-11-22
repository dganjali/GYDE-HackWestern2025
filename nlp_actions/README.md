# NLP Actions - Voice Command System

Voice command processing and action execution system for the companion robot.

## Features

- **Voice Command Recognition**: Process voice commands via web browser
- **Intent Parsing**: Rule-based NLP to understand commands
- **Action Execution**: Execute monitoring, calls, and reminders
- **Reminder Scheduling**: In-memory reminder system for medication alerts

## Setup

1. **Install dependencies:**
   ```bash
   pip install -r nlp_actions/requirements.txt
   ```

2. **Configure contacts** in `nlp_actions/config.py`:
   ```python
   CONTACTS = {
       "daughter": "+1234567890",
       "son": "+1987654321",
       "emergency": "+1555555555",
       "default": "+1234567890",
   }
   ```

3. **Ensure Twilio credentials** are set in your `.env` file (in `twilio_call/twilio_call/`):
   ```
   TWILIO_ACCOUNT_SID=your_account_sid
   TWILIO_AUTH_TOKEN=your_auth_token
   TWILIO_PHONE_NUMBER=+1234567890
   ```

## Running the Server

```bash
# From the repo root
python -m nlp_actions.server
```

The server will start on `http://0.0.0.0:5000`

## Accessing the Voice Client

### On Desktop (localhost):
```
http://localhost:5000/voice_client
```

### On Phone (same network):
1. Find your laptop's local IP address:
   ```bash
   # macOS/Linux
   ifconfig | grep "inet "
   
   # Windows
   ipconfig
   ```

2. Open on phone:
   ```
   http://<laptop_ip>:5000/voice_client
   ```
   Example: `http://192.168.1.100:5000/voice_client`

## Supported Voice Commands

### Monitoring
- "Start monitoring"
- "Begin monitoring"
- "Follow me"
- "Stop monitoring"
- "You can stop"
- "End monitoring"

### Calling
- "Call my daughter"
- "Call my son"
- "Call emergency"
- "Call [contact name]"

### Medication Reminders
- "Remind me to take my medication at 8 pm"
- "Remind me to take my pill at 9:30"
- "Remind me about medication at 20:00"

## API Endpoints

### POST `/voice`
Process a voice command.

**Request:**
```json
{
  "text": "Call my daughter"
}
```

**Response:**
```json
{
  "status": "ok",
  "intent": {
    "action": "CALL_CONTACT",
    "slots": {
      "contact": "daughter"
    }
  },
  "result": {
    "ok": true,
    "action": "CALL_CONTACT",
    "contact_key": "daughter",
    "number": "+1234567890",
    "call_sid": "CAxxxxx"
  }
}
```

### GET `/status`
Get current system status.

**Response:**
```json
{
  "monitoring_state": false,
  "reminders": [
    {
      "time": "20:00",
      "label": "medication",
      "fired": false
    }
  ],
  "status": "ok"
}
```

## Architecture

- **`config.py`**: Configuration constants (contacts, resident info)
- **`intent_parser.py`**: Rule-based NLP intent parsing
- **`action_handler.py`**: Executes parsed intents
- **`reminder_scheduler.py`**: Background reminder scheduling
- **`server.py`**: Flask web server
- **`static/voice_client.html`**: Web-based voice client

## Testing

1. Start the server
2. Open the voice client in a browser
3. Click the microphone button
4. Speak a command like:
   - "Start monitoring"
   - "Call my daughter"
   - "Remind me to take my medication at 8 pm"

Check the server console for logs showing:
- Received text
- Parsed intent
- Action execution results

