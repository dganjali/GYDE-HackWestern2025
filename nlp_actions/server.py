"""
Flask server for NLP actions system.
"""
from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
import threading
import os

import re
from datetime import datetime, timedelta
import threading

# We'll use a simple deterministic parser in this server (rule-based)
from .action_handler import handle_intent
from .config import CONTACTS

from dotenv import load_dotenv

# Ensure environment variables from a .env are loaded before importing Twilio helper
load_dotenv()

# Import the real Twilio caller. If required env vars are missing, the
# import will raise a ValueError from the helper so startup fails loudly —
# this ensures we use the real Twilio integration instead of a demo stub.
from twilio_call.twilio_call.call_alert import trigger_call_for_event

app = Flask(__name__)
CORS(app)  # Enable CORS for cross-origin requests from phone


# -------------------------
# Global in-memory state
# -------------------------
state_lock = threading.Lock()
state = {
    "monitoring": False,
    "reminders": [],  # list of dicts: {id, time: datetime, task, source}
    "next_reminder_id": 1
}

# Contacts are read from `nlp_actions/config.py` via CONTACTS



def get_next_reminder_id():
    with state_lock:
        rid = state["next_reminder_id"]
        state["next_reminder_id"] += 1
        return rid


def call_contact(phone_number: str, reason: str | None) -> None:
    """Call helper: use real Twilio if available, otherwise stub-print.

    This centralizes Twilio usage so the server logs clearly whether a
    real call was attempted or only the demo stub.
    """
    try:
        # Use the high-level helper from twilio_call package.
        sid = trigger_call_for_event(
            to_number=phone_number,
            event_type=(reason or "MANUAL_CALL"),
            resident_name=None,
            location=None,
        )
        print(f"[TWILIO] trigger_call_for_event returned SID={sid}")
        return
    except Exception as e:
        # Do not fall back to a no-op stub — surface the error so the
        # operator can fix credentials or network issues.
        print(f"[TWILIO ERROR] trigger_call_for_event failed: {e}")
        raise


def trigger_reminder(reminder: dict) -> None:
    # This function is called by the threading.Timer when a reminder fires
    print(f"[REMINDER] {reminder['task']} (id={reminder['id']}) at {datetime.now().isoformat()}")
    # TODO: Add WebSocket or push notification to front-end in future


def schedule_relative_reminder(offset_seconds: int, task: str) -> dict:
    target_time = datetime.now() + timedelta(seconds=offset_seconds)
    rid = get_next_reminder_id()
    # Build a time label like '2:30pm'
    time_label = target_time.strftime("%I:%M%p").lstrip('0').lower()
    reminder = {
        "id": rid,
        "time": target_time,
        "task": task,
        "source": "relative",
        "time_label": time_label
    }
    with state_lock:
        state["reminders"].append(reminder)

    delay = max(0, (target_time - datetime.now()).total_seconds())
    timer = threading.Timer(delay, trigger_reminder, args=[reminder])
    timer.daemon = True
    timer.start()
    print(f"[SCHEDULER] Scheduled reminder id={rid} in {delay} seconds for task='{task}'")
    return reminder


def parse_time_to_24h(time_str: str) -> str | None:
    """
    Parse times like:
      - '2:30 pm'
      - '2:30pm'
      - '2 pm'
      - '14:30'
    and return 'HH:MM' in 24h form. If parse fails, return None.
    """
    if not time_str or not isinstance(time_str, str):
        return None

    s = time_str.strip().lower()
    # Normalize common variants
    s = s.replace('.', '')
    s = s.replace('ish', '')
    s = s.replace(' ', ' ').strip()

    # Try several formats
    tried = ["%I:%M %p", "%I %p", "%I%p", "%I:%M%p", "%H:%M", "%H"]
    for fmt in tried:
        try:
            dt = datetime.strptime(s, fmt)
            return dt.strftime("%H:%M")
        except Exception:
            continue

    # As a last resort, try to extract digits like '230pm' or '730'
    m = re.search(r"(\d{1,2})(:?)(\d{2})?\s*(am|pm)?", s)
    if m:
        hour = int(m.group(1))
        minute = int(m.group(3)) if m.group(3) else 0
        ampm = m.group(4)
        if ampm:
            ampm = ampm.lower()
            if ampm == 'pm' and hour < 12:
                hour += 12
            if ampm == 'am' and hour == 12:
                hour = 0
        # Bound check
        if 0 <= hour < 24 and 0 <= minute < 60:
            return f"{hour:02d}:{minute:02d}"

    return None


def schedule_absolute_reminder(time_str_24h: str, task: str, time_label: str = None) -> dict:
    """
    Schedule a reminder at the next occurrence of `time_str_24h` (format 'HH:MM').
    If the time today has passed, schedule for tomorrow.
    """
    try:
        h, m = [int(p) for p in time_str_24h.split(":")]
    except Exception:
        raise ValueError(f"Invalid time format for schedule_absolute_reminder: {time_str_24h}")

    now = datetime.now()
    target_time = now.replace(hour=h, minute=m, second=0, microsecond=0)
    if target_time <= now:
        # schedule for tomorrow
        target_time = target_time + timedelta(days=1)

    rid = get_next_reminder_id()

    # Normalize or compute a time_label like '2:30pm'
    if time_label:
        tl = str(time_label).strip().lower().replace(' ', '')
    else:
        tl = target_time.strftime("%I:%M%p").lstrip('0').lower()

    reminder = {
        "id": rid,
        "time": target_time,
        "task": task,
        "source": "absolute",
        "time_label": tl
    }

    with state_lock:
        state["reminders"].append(reminder)

    delay = max(0, (target_time - datetime.now()).total_seconds())
    timer = threading.Timer(delay, trigger_reminder, args=[reminder])
    timer.daemon = True
    timer.start()
    print(f"[SCHEDULER] Scheduled absolute reminder id={rid} at {target_time.isoformat()} for task='{task}' (in {delay} seconds)")
    return reminder


def format_reminder_time(r: dict) -> str:
    """Return a human-friendly 12-hour time like '2:30pm' (lowercase am/pm, no seconds)."""
    # Prefer explicit time_label when present
    if isinstance(r, dict) and r.get("time_label"):
        return str(r.get("time_label"))

    dt = r.get("time")
    if not isinstance(dt, datetime):
        try:
            dt = datetime.fromisoformat(str(dt))
        except Exception:
            return str(r.get("time"))

    # Format to '%I:%M%p', strip leading zero and lowercase (e.g. '02:30PM' -> '2:30pm')
    label = dt.strftime("%I:%M%p").lstrip('0').lower()
    return label


# -------------------------
# Deterministic rule-based intent parser
# -------------------------
def parse_intent(text: str) -> dict:
    t = (text or "").lower().strip()

    # 1) CALL_CONTACT
    if t.startswith("call ") or " call " in t:
        contact_type = None
        contact_name = None
        reason = None

        if "emergency contact" in t or "emergency" in t:
            contact_type = "emergency"
            reason = "emergency"
        elif "daughter" in t:
            contact_type = "daughter"
        elif "son" in t:
            contact_type = "son"
        elif "caregiver" in t:
            contact_type = "caregiver"
        elif "nurse" in t:
            contact_type = "nurse"
        else:
            m = re.search(r"call\s+([a-zA-Z]+)", t)
            if m:
                name_raw = m.group(1)
                contact_name = name_raw.lower()
                contact_type = "other"

        slots = {"contact_type": contact_type, "contact_name": contact_name, "reason": reason}

        if contact_type == "daughter":
            message = "Okay, I’m calling your daughter now."
        elif contact_type == "son":
            message = "Okay, I’m calling your son now."
        elif contact_type == "emergency":
            message = "I’m calling your emergency contact now."
        elif contact_type in ("caregiver", "nurse"):
            message = f"Okay, I’m calling your {contact_type} now."
        elif contact_type == "other" and contact_name:
            message = f"Okay, I’m calling {contact_name.capitalize()} now."
        else:
            message = "I’m not sure who to call, I could not find that contact."

        return {"intent": "CALL_CONTACT", "slots": slots, "message": message}

    # 2) SET_RELATIVE_REMINDER
    if t.startswith("remind me in "):
        pattern = r"remind me in\s+(\d+)\s*(second|seconds|sec|secs|minute|minutes|min|mins|hour|hours)?"
        m = re.search(pattern, t)
        if m:
            n = int(m.group(1))
            unit = m.group(2) or "seconds"
            unit = unit.lower()
            if unit in ["second", "seconds", "sec", "secs"]:
                offset_seconds = n
            elif unit in ["minute", "minutes", "min", "mins"]:
                offset_seconds = n * 60
            elif unit in ["hour", "hours"]:
                offset_seconds = n * 3600
            else:
                offset_seconds = n

            # Derive task from the remainder of the utterance after the matched time
            remainder = t[m.end():].strip()
            task = None
            if remainder.startswith("to "):
                task = remainder[3:].strip()
            elif remainder:
                task = remainder
            else:
                task = "your reminder"

            slots = {"offset_seconds": offset_seconds, "offset_original": f"in {n} {unit}", "task": task}
            message = f"Okay. I’ll remind you in {n} {unit} to {task}."
            return {"intent": "SET_RELATIVE_REMINDER", "slots": slots, "message": message}

    # 2b) SET_ABSOLUTE_REMINDER
    # Examples: "set a reminder for 2:30 pm to call my doctor"
    #           "remind me at 7 pm to take my meds"
    if t.startswith("set a reminder for ") or t.startswith("remind me at "):
        if t.startswith("set a reminder for "):
            rest = t[len("set a reminder for "):].strip()
        else:
            rest = t[len("remind me at "):].strip()

        # Split on first ' to ' to separate time phrase from task
        time_phrase = rest
        task = None
        if " to " in rest:
            parts = rest.split(" to ", 1)
            time_phrase = parts[0].strip()
            task = parts[1].strip()
        else:
            # If user didn't use 'to', try to remove the time phrase and use the rest
            # e.g. "2:30 pm call my doctor" -> time_phrase='2:30 pm', remainder='call my doctor'
            # If nothing remains, fallback to a neutral label
            # First, attempt to detect the time phrase at the start
            # We'll remove the time_phrase from rest if it appears at the start
            if rest.startswith(time_phrase):
                remainder = rest[len(time_phrase):].strip()
                if remainder:
                    task = remainder
                else:
                    task = "your reminder"
            else:
                # As a last resort, set a neutral fallback
                task = "your reminder"

        time_24 = parse_time_to_24h(time_phrase)
        if time_24:
            slots = {"time": time_24, "time_original": time_phrase, "task": task}
            message = f"Got it. I\u2019ll remind you at {time_phrase} to {task}."
            return {"intent": "SET_ABSOLUTE_REMINDER", "slots": slots, "message": message}


    # 3) START_MONITORING
    if t == "start monitoring" or t == "follow me" or "start monitoring" in t:
        slots = {"reason": None}
        message = "I’ve started monitoring. I’ll stay with you and watch for any problems."
        return {"intent": "START_MONITORING", "slots": slots, "message": message}

    # 4) STOP_MONITORING
    if t == "stop monitoring" or "stop following" in t:
        slots = {"reason": None}
        message = "Okay, I’ve stopped active monitoring. I’ll still be nearby if you need me."
        return {"intent": "STOP_MONITORING", "slots": slots, "message": message}

    # 5) QUERY_STATUS
    if "status" in t or "what is the server status" in t or "is monitoring on" in t:
        slots = {"include_monitoring_state": True, "include_reminders": True}
        message = "I’ll check your monitoring and reminders now."
        return {"intent": "QUERY_STATUS", "slots": slots, "message": message}

    # 6) QUERY_REMINDERS
    if ("reminders" in t) and ("do i have" in t or "what reminders" in t or "my reminders" in t or t.startswith("do i have")):
        slots = {"scope": "all", "time": None}
        message = "I’ll check your reminders now and read them out."
        return {"intent": "QUERY_REMINDERS", "slots": slots, "message": message}

    # DEFAULT
    return {
        "intent": "UNKNOWN",
        "slots": {"raw_text": text},
        "message": "I’m not sure I understood that. You can ask me to call someone, start monitoring, stop monitoring, set a reminder, or check status."
    }


@app.route("/voice", methods=["POST"])
def voice():
    """Handle voice command POST requests."""
    try:
        data = request.get_json(force=True, silent=True) or {}
        text = data.get("text", "")

        print(f"[VOICE] Received text: {text!r}")

        intent_data = parse_intent(text)
        intent = intent_data.get("intent")
        slots = intent_data.get("slots", {})
        message = intent_data.get("message", "")

        # Execute behavior for the five core intents
        if intent == "CALL_CONTACT":
            contact_type = slots.get("contact_type")
            contact_name = slots.get("contact_name")
            reason = slots.get("reason")

            phone_number = None
            # 1) If contact_type is one of the explicit keys, use it
            if contact_type in ["daughter", "son", "emergency"]:
                phone_number = CONTACTS.get(contact_type)

            # 2) Skip named-contact resolution for demo (optional later)

            # 3) Fallback to default if no specific match
            if phone_number is None:
                phone_number = CONTACTS.get("default")

            if phone_number is None:
                ok = False
                message = "I couldn\'t find a number to call."
            else:
                call_contact(phone_number, reason)
                ok = True
                # If we didn't have a direct mapping for the requested contact, inform user
                if contact_type not in ["daughter", "son", "emergency"]:
                    message = "I couldn\'t find that specific contact, so I\'m calling your default contact."

            return jsonify({"ok": ok, "intent": intent, "slots": slots, "message": message})

        elif intent == "SET_RELATIVE_REMINDER":
            offset_seconds = int(slots.get("offset_seconds", 0))
            task = slots.get("task", "do that")
            reminder = schedule_relative_reminder(offset_seconds, task)
            ok = True
            return jsonify({"ok": ok, "intent": intent, "slots": slots, "message": message})

        elif intent == "SET_ABSOLUTE_REMINDER":
            time_24h = slots.get("time")
            task = slots.get("task") or "your reminder"
            time_original = slots.get("time_original")

            if not time_24h:
                ok = False
                message = "I\'m sorry, I couldn\'t understand the time for that reminder."
            else:
                try:
                    reminder = schedule_absolute_reminder(time_24h, task, time_label=time_original)
                    ok = True
                except Exception as e:
                    ok = False
                    message = f"Failed to schedule reminder: {e}"

            return jsonify({"ok": ok, "intent": intent, "slots": slots, "message": message})

        elif intent == "QUERY_REMINDERS":
            with state_lock:
                reminders = list(state.get("reminders", []))

            if not reminders:
                summary = "You have no reminders."
            else:
                reminders_sorted = sorted(reminders, key=lambda r: r["time"])
                phrases = []
                for r in reminders_sorted:
                    t_str = format_reminder_time(r)
                    phrases.append(f"at {t_str} {r['task']}")
                summary = "You have " + str(len(reminders_sorted)) + " reminder(s): " + ", ".join(phrases) + "."

            message = summary
            ok = True
            return jsonify({"ok": ok, "intent": intent, "slots": slots, "message": message})

        elif intent == "START_MONITORING":
            with state_lock:
                state["monitoring"] = True
            print("[MONITORING] Started")
            ok = True
            return jsonify({"ok": ok, "intent": intent, "slots": slots, "message": message})

        elif intent == "STOP_MONITORING":
            with state_lock:
                state["monitoring"] = False
            print("[MONITORING] Stopped")
            ok = True
            return jsonify({"ok": ok, "intent": intent, "slots": slots, "message": message})

        elif intent == "QUERY_STATUS":
            with state_lock:
                monitoring = state.get("monitoring", False)
                reminders = list(state.get("reminders", []))

            if not monitoring and not reminders:
                summary = "Monitoring is OFF. You have no reminders."
            else:
                parts = []
                if monitoring:
                    parts.append("Monitoring is ON.")
                else:
                    parts.append("Monitoring is OFF.")
                if reminders:
                    reminders_sorted = sorted(reminders, key=lambda r: r["time"])
                    reminder_phrases = []
                    for r in reminders_sorted:
                        t_str = format_reminder_time(r)
                        reminder_phrases.append(f"at {t_str} {r['task']}")
                    parts.append("You have " + str(len(reminders_sorted)) + " reminder(s): " + ", ".join(reminder_phrases) + ".")
                else:
                    parts.append("You have no reminders.")
                summary = " ".join(parts)

            message = summary
            ok = True
            return jsonify({"ok": ok, "intent": intent, "slots": slots, "message": message})

        else:
            # UNKNOWN
            return jsonify({"ok": False, "intent": intent, "slots": slots, "message": message})

    except Exception as e:
        print(f"[VOICE] Error processing request: {e}")
        return jsonify({"ok": False, "intent": "ERROR", "slots": {}, "message": str(e)}), 500


@app.route("/voice_client")
def voice_client():
    """Serve the voice client HTML page."""
    base_dir = os.path.dirname(os.path.abspath(__file__))
    static_dir = os.path.join(base_dir, "static")
    return send_from_directory(static_dir, "voice_client.html")


@app.route("/status", methods=["GET"])
def status():
    """Get current system status."""
    with state_lock:
        monitoring = state.get("monitoring", False)
        reminders = [
            {"id": r["id"], "time": r["time"].isoformat(), "task": r["task"], "source": r.get("source", ""), "time_label": r.get("time_label")}
            for r in state.get("reminders", [])
        ]

    return jsonify({
        "monitoring_state": monitoring,
        "reminders": reminders,
        "status": "ok"
    })


if __name__ == "__main__":
    print("[SERVER] Starting Flask server on http://0.0.0.0:5000")
    print("[SERVER] Voice client available at: http://localhost:5000/voice_client")
    print("[SERVER] Status endpoint: http://localhost:5000/status")
    app.run(host="0.0.0.0", port=5000, debug=True)

