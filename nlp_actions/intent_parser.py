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
        return {"action": "START_MONITORING", "slots": {}}
    
    # STOP_MONITORING
    if any(phrase in text for phrase in ["stop monitoring", "you can stop", "stop following", "end monitoring"]):
        return {"action": "STOP_MONITORING", "slots": {}}
    
    # CALL_CONTACT
    if "call" in text:
        contact_key = None
        for alias, canonical in CONTACT_ALIASES.items():
            if alias in text:
                contact_key = canonical
                break
        
        if contact_key is None:
            contact_key = "default"
        
        return {"action": "CALL_CONTACT", "slots": {"contact": contact_key}}
    
    # SET_MEDICATION_REMINDER
    if ("remind me" in text or "reminder" in text) and ("medication" in text or "pill" in text or "medicine" in text):
        time_str = None
        
        # Try to match time patterns
        # Pattern 1: "8 pm", "9 am", "10 pm", etc.
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
        else:
            # Pattern 2: "20:00", "9:30", etc.
            time_pattern = r'(\d{1,2}):(\d{2})'
            match = re.search(time_pattern, text)
            if match:
                hour = int(match.group(1))
                minute = match.group(2)
                time_str = f"{hour:02d}:{minute}"
        
        return {
            "action": "SET_MEDICATION_REMINDER",
            "slots": {"time": time_str, "label": "medication"}
        }
    
    # Fallback
    return {"action": "UNKNOWN", "slots": {}}

