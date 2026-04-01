/**
 * blink.ino — Non-blocking LED blink using millis()
 *
 * Blinks the built-in LED at a configurable interval without using delay(),
 * keeping the main loop free to handle other tasks concurrently.
 *
 * Board:  Arduino Uno (or any board with LED_BUILTIN)
 * Author: Power Platform workspace starter
 *
 * Wiring:
 *   No external components required. Uses the on-board LED (pin 13 on Uno).
 *   To drive an external LED: connect anode → pin LED_PIN via 220Ω resistor,
 *   cathode → GND.
 */

// ── Pin definitions ──────────────────────────────────────────────────────────

const uint8_t LED_PIN = LED_BUILTIN;

// ── Configuration ────────────────────────────────────────────────────────────

const uint32_t BLINK_INTERVAL_MS = 500;  // ms between state toggles (on→off or off→on)

// ── State ────────────────────────────────────────────────────────────────────

namespace blink {
  bool          ledState      = LOW;
  uint32_t      lastToggleMs  = 0;
}

// ── Helpers ──────────────────────────────────────────────────────────────────

/**
 * Toggle the LED if the blink interval has elapsed.
 * Must be called every loop iteration to function correctly.
 */
void updateBlink() {
  const uint32_t now = millis();

  if (now - blink::lastToggleMs >= BLINK_INTERVAL_MS) {
    blink::lastToggleMs = now;
    blink::ledState     = !blink::ledState;
    digitalWrite(LED_PIN, blink::ledState);

#ifdef DEBUG
    Serial.print(F("[blink] LED "));
    Serial.println(blink::ledState ? F("ON") : F("OFF"));
#endif
  }
}

// ── Arduino lifecycle ─────────────────────────────────────────────────────────

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial) { /* wait for USB serial on Leonardo / Micro */ }
  Serial.println(F("[blink] setup complete"));
#endif

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  updateBlink();

  // Add other non-blocking tasks here — they will run concurrently
  // because updateBlink() never blocks.
}
