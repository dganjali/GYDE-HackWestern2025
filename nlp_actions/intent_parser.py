"""
Rule-based NLP intent parser for voice commands.
"""
import re


CONTACT_ALIASES = {
    "daughter": "daughter",
    "my daughter": "daughter",
    "daughter's": "daughter",
    "son": "son",
    "my son": "son",
    "son's": "son",
    "emergency contact": "emergency",
    "emergency": "emergency",
    "john": "son",  # Example name mapping
}


def parse_intent(text: str) -> dict:
    """
    Parse raw speech transcript into structured intent.
    
    Args:
        text: Raw speech transcript string
        
    Returns:
        dict with "action" and "slots" keys
    """
    # Normalize
    text = text.lower().strip()
    
    # START_MONITORING
    if any(phrase in text for phrase in ["start monitoring", "begin monitoring", "follow me", "start following"]):
        return {
            "action": "START_MONITORING",
            "intent": "START_MONITORING",
            "slots": {"reason": None},
            "user_tts": "I\'ve started monitoring. I\'ll stay with you and watch for any problems.",
            "caregiver_alert": {"level": "info", "text": "User started monitoring."}
        }
    
    # STOP_MONITORING
    if any(phrase in text for phrase in ["stop monitoring", "you can stop", "stop following", "end monitoring"]):
        return {
            "action": "STOP_MONITORING",
            "intent": "STOP_MONITORING",
            "slots": {"reason": None},
            "user_tts": "Monitoring stopped. I\'ll stay nearby but not actively monitoring.",
            "caregiver_alert": {"level": "info", "text": "User stopped monitoring."}
        }
    
    # CALL_CONTACT
    if "call" in text:
        # Determine contact type and name
        contact_type = None
        contact_name = None
        reason = None

        for alias, canonical in CONTACT_ALIASES.items():
            if alias in text:
                contact_type = canonical
                break

        # If no alias, try to capture a name after the word "call"
        if contact_type is None:
            m = re.search(r'call\s+(?:my\s+)?([a-zA-Z\-\']+)', text)
            if m:
                name = m.group(1)
                contact_name = name
                contact_type = 'other'
        
        if 'emergency' in text:
            reason = 'emergency'
            contact_type = contact_type or 'emergency'

        if contact_type is None:
            contact_type = 'default'

        slots = {
            'contact_type': contact_type,
            'contact_name': contact_name,
            'reason': reason
        }

        # user-readable confirmation
        if contact_type == 'other' and contact_name:
            user_tts = f"Okay, calling {contact_name} now."
        elif contact_type == 'default':
            user_tts = "Okay, calling your default contact now."
        else:
            user_tts = f"Calling your {contact_type} now."

        caregiver_alert = {'level': 'info' if reason != 'emergency' else 'emergency', 'text': ''}

        return {
            'action': 'CALL_CONTACT',
            'intent': 'CALL_CONTACT',
            'slots': slots,
            'user_tts': user_tts,
            'caregiver_alert': caregiver_alert
        }
    
    # SET_MEDICATION_REMINDER / relative reminders (supports seconds for demo)
    if ("remind me" in text or "reminder" in text) and ("medication" in text or "pill" in text or "medicine" in text):
        time_str = None
        slots = {}
        intent_name = "SET_MEDICATION_REMINDER"

        # Pattern: relative times like "in 10 seconds", "in 5 minutes", "in 2 hours"
        rel_pattern = r'in\s+(\d+)\s*(second|seconds|sec|secs|minute|minutes|min|mins|hour|hours)'
        rel_match = re.search(rel_pattern, text)
        if rel_match:
            qty = int(rel_match.group(1))
            unit = rel_match.group(2)
            unit = unit.lower()
            if unit.startswith('second') or unit.startswith('sec'):
                offset_seconds = qty
            elif unit.startswith('minute') or unit.startswith('min'):
                offset_seconds = qty * 60
            elif unit.startswith('hour'):
                offset_seconds = qty * 3600
            else:
                offset_seconds = qty

            slots = {
                "offset_seconds": offset_seconds,
                "offset_original": rel_match.group(0),
                "task": "medication"
            }
            # Provide user-facing time string for TTS
            user_tts = f"Okay. I'll remind you in {qty} {unit} to take your medication."
            caregiver_alert = {"level": "none", "text": ""}

            return {
                "action": "SET_MEDICATION_REMINDER",
                "intent": "SET_RELATIVE_REMINDER",
                "slots": slots,
                "user_tts": user_tts,
                "caregiver_alert": caregiver_alert
            }

        # Try to match absolute time patterns like "8 pm", "9 am", "10 pm"
        am_pm_pattern = r'(\d{1,2})\s*(am|pm)'
        match = re.search(am_pm_pattern, text)
        if match:
            hour = int(match.group(1))
            period = match.group(2).lower()

            if period == "pm" and hour != 12:
                hour += 12
            elif period == "am" and hour == 12:
                hour = 0

            time_str = f"{hour:02d}:00"
            slots = {"time": time_str, "time_original": match.group(0), "task": "medication"}
            user_tts = f"Got it. I'll remind you at {match.group(0)} to take your medication."
            caregiver_alert = {"level": "none", "text": ""}

            return {
                "action": "SET_MEDICATION_REMINDER",
                "intent": "SET_ABSOLUTE_REMINDER",
                "slots": slots,
                "user_tts": user_tts,
                "caregiver_alert": caregiver_alert
            }
        else:
            # Pattern 2: "20:00", "9:30", etc.
            time_pattern = r'(\d{1,2}):(\d{2})'
            match = re.search(time_pattern, text)
            if match:
                hour = int(match.group(1))
                minute = match.group(2)
                time_str = f"{hour:02d}:{minute}"
                slots = {"time": time_str, "time_original": match.group(0), "task": "medication"}
                user_tts = f"Got it. I'll remind you at {match.group(0)} to take your medication."
                caregiver_alert = {"level": "none", "text": ""}

                return {
                    "action": "SET_MEDICATION_REMINDER",
                    "intent": "SET_ABSOLUTE_REMINDER",
                    "slots": slots,
                    "user_tts": user_tts,
                    "caregiver_alert": caregiver_alert
                }

        # If no time parsed, return unknown reminder intent
        return {
            "action": "SET_MEDICATION_REMINDER",
            "intent": "UNKNOWN",
            "slots": {},
            "user_tts": "I didn't catch the time for the reminder. When would you like me to remind you?",
            "caregiver_alert": {"level": "none", "text": ""}
        }
    
    # Fallback
    return {
        "action": "UNKNOWN",
        "intent": "UNKNOWN",
        "slots": {"raw_text": text},
        "user_tts": "Sorry, I did not understand that. Please try again.",
        "caregiver_alert": {"level": "none", "text": ""}
    }

