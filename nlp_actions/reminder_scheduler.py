"""
Simple in-memory reminder scheduler.
"""
import time
import threading
from datetime import datetime
from typing import Callable, List, Dict


reminders: List[Dict] = []
reminders_lock = threading.Lock()


def add_reminder(time_str: str, label: str) -> None:
    """
    Add a reminder to the schedule.
    
    Args:
        time_str: Time in HH:MM format (24-hour)
        label: Label for the reminder
    """
    with reminders_lock:
        reminders.append({
            "time": time_str,
            "label": label,
            "fired": False
        })
        print(f"[SCHEDULER] Added reminder: {label} at {time_str}")


def get_reminders() -> List[Dict]:
    """
    Get all active reminders.
    
    Returns:
        List of reminder dicts
    """
    with reminders_lock:
        return [r for r in reminders if not r.get("fired", False)]


def start_scheduler(callback: Callable[[Dict], None]) -> None:
    """
    Start the reminder scheduler in a background loop.
    
    Args:
        callback: Function to call when a reminder fires
    """
    print("[SCHEDULER] Starting reminder scheduler...")
    
    while True:
        try:
            current_time = datetime.now().strftime("%H:%M")
            
            with reminders_lock:
                for reminder in reminders:
                    if not reminder.get("fired", False) and reminder["time"] == current_time:
                        print(f"[SCHEDULER] Firing reminder: {reminder}")
                        reminder["fired"] = True
                        try:
                            callback(reminder)
                        except Exception as e:
                            print(f"[SCHEDULER] Error in callback: {e}")
            
            # Clean up old fired reminders (older than 1 hour)
            with reminders_lock:
                current_hour = datetime.now().hour
                reminders[:] = [
                    r for r in reminders
                    if not r.get("fired", False) or 
                    (r.get("fired", False) and 
                     (int(r["time"].split(":")[0]) == current_hour or 
                      int(r["time"].split(":")[0]) == (current_hour - 1) % 24))
                ]
        
        except Exception as e:
            print(f"[SCHEDULER] Error in scheduler loop: {e}")
        
        # Check every 30 seconds
        time.sleep(30)

