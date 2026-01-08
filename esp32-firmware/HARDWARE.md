Hardware recommendations for relay and power protection

- Use proper relay drivers (MOSFETs or driver IC) when driving loads. Avoid driving relays directly from ESP GPIO if current exceeds pin limits.
- Use flyback diodes across inductive loads (coils/relays) on the driver side.
- Use snubber RC networks (e.g., 100nF + 100Ω) across relay contacts when switching inductive loads.
- Use TVS diodes and bulk decoupling capacitors on supply lines to protect against transients.
- Consider opto-isolators or driver modules for isolating ESP from relay coil noise.
- Use separate 5V supply for relays (if modules require 5V) and common ground; ensure supply can handle inrush.
- Add an NTC or soft-start for large inductive loads or motors.
- For long-life switches: avoid rapid on/off cycling; implement minimum on/off durations in firmware (done).
- Avoid PWM on mechanical relays; use SSR for fast switching or PWM control.

Software-side guidance (already applied):
- Software rate-limiting: `RELAY_MIN_TOGGLE_MS` prevents rapid toggling (default 5s).
- Enforce cool-downs for fan and pump where needed.
- Debounce and limit schedule additions to avoid frequent writes.

Testing tips:
- Test under worst-case loads (motor start, stall current) and monitor supply voltage dips.
- Use an oscilloscope to inspect switching transients at relay contacts and coil drive.
- Measure ESP ground bounce and add star-grounding if necessary.

