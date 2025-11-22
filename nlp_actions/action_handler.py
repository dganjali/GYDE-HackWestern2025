"""
Action handler - executes parsed intents.
"""
import sys
import os

# Add parent directory to path to import from twilio_call
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from .config import CONTACTS, RESIDENT_NAME, RESIDENT_LOCATION
from .reminder_scheduler import add_reminder

try:
    # Try the actual path structure
    from twilio_call.twilio_call.call_alert import trigger_call_for_event
except ImportError:
    try:
        # Try alternative import path
        import sys
        import os
        twilio_call_path = os.path.join(os.path.dirname(__file__), '..', 'twilio_call', 'twilio_call')
        if os.path.exists(twilio_call_path):
            sys.path.insert(0, os.path.dirname(twilio_call_path))
            from twilio_call.call_alert import trigger_call_for_event
        else:
            raise ImportError("twilio_call module not found")
    except ImportError:
        print("[WARNING] Could not import trigger_call_for_event. Call actions will not work.")
        print("[WARNING] Make sure twilio_call module is accessible. Using mock function.")
        def trigger_call_for_event(*args, **kwargs):
            print(f"[MOCK] Would call: {args}, {kwargs}")
            return "mock_call_sid"


# Global monitoring state
monitoring_state = False


def handle_intent(intent: dict) -> dict:
    """
    Takes intent dict from parse_intent and performs the required action.
    
    Args:
        intent: Intent dict with "action" and "slots" keys
        
    Returns:
        Result dict for logging/response
    """
    global monitoring_state
    action = intent.get("action", "UNKNOWN")
    slots = intent.get("slots", {})
    
    if action == "START_MONITORING":
        monitoring_state = True
        print(f"[ACTION] Monitoring started. State: {monitoring_state}")
        return {
            "ok": True,
            "action": "START_MONITORING",
            "monitoring_state": monitoring_state
        }
    
    elif action == "STOP_MONITORING":
        monitoring_state = False
        print(f"[ACTION] Monitoring stopped. State: {monitoring_state}")
        return {
            "ok": True,
            "action": "STOP_MONITORING",
            "monitoring_state": monitoring_state
        }
    
    elif action == "CALL_CONTACT":
        contact_key = slots.get("contact", "default")
        number = CONTACTS.get(contact_key, CONTACTS["default"])
        
        print(f"[ACTION] Calling {contact_key} at {number}")
        
        try:
            call_sid = trigger_call_for_event(
                to_number=number,
                event_type="MANUAL_CALL",
                resident_name=RESIDENT_NAME,
                location=RESIDENT_LOCATION,
            )
            return {
                "ok": True,
                "action": "CALL_CONTACT",
                "contact_key": contact_key,
                "number": number,
                "call_sid": call_sid
            }
        except Exception as e:
            error_msg = str(e)
            print(f"[ACTION] Error making call: {error_msg}")
            
            # Provide helpful error message for unverified numbers
            if "unverified" in error_msg.lower() or "21219" in error_msg:
                helpful_msg = (
                    f"Phone number {number} is not verified in your Twilio account. "
                    "Trial accounts can only call verified numbers. "
                    "Please verify this number at: https://console.twilio.com/us1/develop/phone-numbers/manage/verified"
                )
                print(f"[ACTION] {helpful_msg}")
                return {
                    "ok": False,
                    "action": "CALL_CONTACT",
                    "error": helpful_msg,
                    "number": number
                }
            
            return {
                "ok": False,
                "action": "CALL_CONTACT",
                "error": error_msg
            }
    
    elif action == "SET_MEDICATION_REMINDER":
        time_str = slots.get("time")
        label = slots.get("label", "medication")
        
        if time_str is not None:
            add_reminder(time_str, label)
            return {
                "ok": True,
                "action": "SET_MEDICATION_REMINDER",
                "time": time_str,
                "label": label
            }
        else:
            return {
                "ok": False,
                "action": "SET_MEDICATION_REMINDER",
                "error": "No time parsed from request"
            }
    
    elif action == "UNKNOWN":
        return {
            "ok": False,
            "action": "UNKNOWN"
        }
    
    else:
        return {
            "ok": False,
            "action": action,
            "error": "Unknown action type"
        }


def get_monitoring_state() -> bool:
    """Get current monitoring state."""
    return monitoring_state

