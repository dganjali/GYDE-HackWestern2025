"""
Flask server for NLP actions system.
"""
from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
import threading
import os

from .intent_parser import parse_intent
from .action_handler import handle_intent, get_monitoring_state
from .reminder_scheduler import start_scheduler, get_reminders

app = Flask(__name__)
CORS(app)  # Enable CORS for cross-origin requests from phone


@app.route("/voice", methods=["POST"])
def voice():
    """Handle voice command POST requests."""
    try:
        data = request.get_json(force=True)
        text = data.get("text", "")
        
        print(f"[VOICE] Received text: {text!r}")
        
        intent = parse_intent(text)
        print(f"[VOICE] Parsed intent: {intent}")
        
        result = handle_intent(intent)
        
        return jsonify({
            "status": "ok",
            "intent": intent,
            "result": result,
        })
    except Exception as e:
        print(f"[VOICE] Error processing request: {e}")
        return jsonify({
            "status": "error",
            "error": str(e)
        }), 500


@app.route("/voice_client")
def voice_client():
    """Serve the voice client HTML page."""
    base_dir = os.path.dirname(os.path.abspath(__file__))
    static_dir = os.path.join(base_dir, "static")
    return send_from_directory(static_dir, "voice_client.html")


@app.route("/status", methods=["GET"])
def status():
    """Get current system status."""
    return jsonify({
        "monitoring_state": get_monitoring_state(),
        "reminders": get_reminders(),
        "status": "ok"
    })


def reminder_callback(reminder):
    """Callback function when a reminder fires."""
    print(f"[REMINDER] Time reached for: {reminder}")
    # TODO: Could trigger a call or notification here
    # For now, just log it


if __name__ == "__main__":
    # Start reminder scheduler in background thread
    scheduler_thread = threading.Thread(
        target=start_scheduler,
        args=(reminder_callback,),
        daemon=True
    )
    scheduler_thread.start()
    
    print("[SERVER] Starting Flask server on http://0.0.0.0:5000")
    print("[SERVER] Voice client available at: http://localhost:5000/voice_client")
    print("[SERVER] Status endpoint: http://localhost:5000/status")
    
    app.run(host="0.0.0.0", port=5000, debug=True)

