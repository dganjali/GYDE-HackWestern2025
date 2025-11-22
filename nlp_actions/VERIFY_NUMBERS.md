# How to Verify Phone Numbers in Twilio

## The Problem
Twilio trial accounts can **only call verified phone numbers**. If you see an error like:
```
Unable to create record: The number +1XXXXXXXXXX is unverified. 
Trial accounts may only make calls to verified numbers.
```

You need to verify the phone numbers you want to call.

## Solution: Verify Your Phone Numbers

### Step 1: Go to Twilio Console
1. Open https://console.twilio.com/
2. Log in to your account

### Step 2: Navigate to Verified Caller IDs
1. Click on **Phone Numbers** in the left sidebar
2. Click on **Manage** → **Verified Caller IDs**
   - Or go directly to: https://console.twilio.com/us1/develop/phone-numbers/manage/verified

### Step 3: Add a Phone Number
1. Click **Add a new Caller ID** or **Verify a number**
2. Enter the phone number you want to verify (e.g., `+16475194566`)
3. Choose verification method:
   - **Call me** - Twilio will call you with a verification code
   - **Text me** - Twilio will text you a verification code
4. Enter the verification code when prompted
5. Click **Verify**

### Step 4: Update Your Config
Make sure the numbers in `nlp_actions/config.py` match your verified numbers:

```python
CONTACTS = {
    "daughter": "+14379822952",  # Must be verified
    "son": "+16475194566",       # Must be verified
    "emergency": "+14379822952", # Must be verified
    "default": "+16475194566",   # Must be verified
}
```

## Current Numbers in Your Config
- **Daughter**: `+14379822952`
- **Son**: `+16475194566`
- **Emergency**: `+14379822952`
- **Default**: `+16475194566`

**Action Required**: Verify both `+14379822952` and `+16475194566` in your Twilio console.

## Alternative: Upgrade Your Account
If you need to call unverified numbers, you can upgrade your Twilio account from trial to paid. However, verifying numbers is free and recommended for testing.

## Testing After Verification
Once numbers are verified, try your voice command again:
- Say "call my son" or type it in the text input
- The call should now work!

