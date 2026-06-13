# Felix back-in auto-charging dock (USB-C PD) — BOM & build guide

A 3D-printed dock the mecanum robot **backs into** to recharge its SmallRig
V-mount battery automatically. The battery already charges from a standard USB-C
PD charger (confirmed: a MacBook USB-C charger plugged into the battery's USB-C
port charges it). This dock simply **presents a USB-C PD charger on spring
contacts** so the robot can connect to it by backing in — no human plugging in.

> **Why this is the simple/safe path:** the battery is a USB-C PD *sink* with its
> own internal charge controller + BMS. It does all the CC-CV, cell balancing,
> termination, and over-voltage protection itself. The dock is therefore *just a
> PD brick on pogo pins* — no 16.8 V supply, no charge relay, no sense-MCU, no
> reverse-polarity FET. USB-C's own handshake provides "dead until mated."

---

## 1. How it works

Electrically this is **one USB-C-to-USB-C charging cable, cut in half, with a
pad/pogo contact interface spliced into the middle.** Three conductors cross the
gap: **VBUS, GND, CC.**

```
 DOCK (fixed)                                  ROBOT (tail)                BATTERY
 ┌─────────────┐                               ┌──────────────┐           ┌─────────┐
 │ USB-C PD    │   USB-C    ┌────────┐  pogo    │ pad: VBUS ───┼── fuse ──┐│ USB-C   │
 │ brick (65W+)│──────────► │breakout│══════════│ pad: GND ────┼──────────┼┤ port    │
 │ = SOURCE    │  (cable)   │VBUS/GND│  pogo    │ pad: CC ─────┼──────────┘│(= SINK, │
 └─────────────┘            │  /CC   │══════════│              │  pigtail   │ internal│
                            └────────┘  pogo    └──────────────┘            │ charger)│
                                                                            └─────────┘
```

**The cycle, fully automatic:**

```
1. Battery low → robot navigates to the dock (Nav2/AMCL; optional ArUco marker
   on the dock for a precise final line-up).
2. Robot backs in slowly.
3. Printed FUNNEL + chamfers correct yaw/lateral error → tail pads land on the
   dock pogo pins  (COARSE alignment; the wide pads do the rest).
4. CC pad connects → the PD brick sees the battery's pull-down (Rd) on CC →
   enables 5 V → PD negotiates → ramps to ~20 V → the BATTERY'S OWN charger
   does CC-CV to the cells. Identical to plugging the MacBook charger in.
5. Charge runs; the battery's BMS terminates + balances at full.
6. Undock: robot drives forward → pads separate → CC opens → brick kills VBUS.
```

**Built-in safety (all from USB-C itself, nothing to build):**

- **Dock pads are dead until mated.** A Type-C source holds VBUS at 0 V until it
  detects the sink's Rd resistor on CC. No robot seated → CC open → no power.
- **Exposed tail pads are inherently safe too.** The battery (a bidirectional
  USB-C port) will not *output* power either unless it detects a valid sink (Rd)
  on CC. A dumb conductive bridge across the VBUS/GND tail pads (CC floating)
  presents no Rd → the battery sources nothing. *(Still fuse the pigtail — belt
  and suspenders — but the inherent behavior is the real protection.)*
- **Wrong-orientation = no charge, not damage.** If the robot seats skewed and
  the pads don't map correctly, CC won't connect → no handshake → no power.
  Make the funnel asymmetric so only the correct orientation fully seats.

---

## 2. Bill of materials

### 2a. Electrical

| # | Part | Spec | Qty | ~USD | Notes |
|---|------|------|-----|------|-------|
| A1 | **USB-C PD brick** | 65 W+ (a spare MacBook/laptop charger works) | 1 | 0–30 | The charge source. Lives in/at the dock. |
| A2 | USB-C **female** breakout board | exposes VBUS/GND/CC1/CC2 solder pads | 1 | 3 | The brick's cable plugs into this on the dock. |
| A3 | USB-C **male** breakout (or a cut USB-C cable end) | exposes VBUS/GND/CC1/CC2 | 1 | 3 | Plugs into the **battery's** USB-C; pigtail to the tail pads. |
| A4 | Inline fuse + holder | 5 A automotive blade, on VBUS | 1 | 2 | At the battery end of the pigtail. |
| A5 | *(optional)* e-marker emulator | 5 A / 100 W e-marker chip | 1 | 3 | Add on the dock side **only if you want >60 W** charging. Skip for 60 W. |

> Without an e-marker the spliced path is limited to **3 A ≈ 60 W** (USB-C rule:
> 5 A/100 W needs an e-marker in the cable). 60 W fully charges a 99–155 Wh pack
> in ~1.5–2.5 h — fine for a dock. Add A5 only if you want faster.

### 2b. Contacts & wiring

| # | Part | Spec | Qty | ~USD | Notes |
|---|------|------|-----|------|-------|
| B1 | Power pogo pins | 2–3 mm dia, **gold-plated**, ≥5 A, 1–2 mm stroke | 4 | 6–12 | **2 in parallel** each for VBUS and GND (thermal margin/redundancy). |
| B2 | Signal pogo pin | small gold pogo, ≥1 A | 1 | 1 | CC line. |
| B3 | Tail bus-bar pads | tin/gold-plated copper, **20–30 mm wide** | 2 | 3 | VBUS + GND. Wide so the pin lands anywhere. |
| B4 | Tail CC pad | small plated pad, **slightly recessed/shorter** | 1 | 1 | CC — see make/break sequencing in §4. |
| B5 | Wire | 18–20 AWG (power), 24–26 AWG (CC) | — | 3 | 18 AWG is ample for 3 A. |
| B6 | Heatshrink, ferrules, terminals | — | — | 2 | Strain-relieve every joint. |

> *Upgrade alt (no exposed pads):* a **magnetic USB-C breakaway adapter that
> passes full PD** replaces B1–B4 — it self-aligns the last 1–2 mm and breaks
> away cleanly, with no exposed copper on the robot. Downside: a small, fussier
> alignment target (~±1–2 mm) vs. the forgiving wide pads (~±10–15 mm). Pads are
> recommended for a reliable unattended dock.

### 2c. Mechanical / printed

| # | Part | Spec | Qty | ~USD | Notes |
|---|------|------|-----|------|-------|
| C1 | Printed dock body w/ funnel | PETG/ABS (heat-tolerant) | 1 | filament | 30° half-angle funnel, 50–80 mm mouth. |
| C2 | Floating pogo bracket | printed, ±5 mm X/Y travel | 1 | filament | Spring-centered; absorbs residual misalignment. |
| C3 | Centering springs | light compression, ~5 mm | 2–4 | 2 | Center the floating bracket. |
| C4 | M3 hardware + heat-set inserts | screws, inserts, T-slot | kit | 5 | Mount pogo block, lid, wall bracket. |
| C5 | Rubber feet / wall bracket | — | 1 | 3 | Keep the dock from sliding on back-in. |
| C6 | *(optional)* ArUco marker / reflective tape | — | 1 | 1 | Precise Nav2 final-approach reference. |

**Rough total:** ~$30–60 (less if you already own the PD brick).

---

## 3. Alignment & mechanical design

Alignment happens in **layers** — the robot never has to be millimeter-accurate;
the funnel does the precision work:

```
Robot nav / back-in     ±2–3 cm    (Nav2/AMCL, mecanum strafe to pre-align)
   ↓ must land inside…
Funnel mouth            ±3–4 cm    (mouth 50–80 mm wide — generous)
   ↓ taper converges to…
Seat at hard stop       ±1–2 mm    (funnel mechanically forces this)
   ↓ float bracket absorbs residual, pads have ±10–15 mm of their own slop
Contacts mate
```

Targets (from AMR/consumer-robot dock prior art):

- **Funnel:** 30° half-angle taper, 50–80 mm mouth, **45° chamfer on every entry
  edge** so nothing catches on an angled approach. Tolerates ±5–8° approach yaw.
- **Floating pogo block:** ±5 mm X/Y spring-centered travel (T-slot + 2 springs).
- **Seat & stroke:** funnel depth seats the **chassis against a hard stop** with
  ~1 mm pogo compression remaining — the hard stop takes the docking force, never
  the pins (pins need 0.5–2 mm working stroke, never bottomed).
- **Pads:** 20–30 mm wide laterally, plated (bare copper oxidizes, resistance
  climbs). Pin layout L→R: `[ VBUS ][ VBUS ][ GND ][ GND ][ CC ]`.
- **Polarity/orientation keying:** make the funnel/seat **asymmetric** so the
  robot can only fully seat one way. (Belt-and-suspenders; CC handshake already
  refuses to power a mis-mapped seat.)
- Print PETG/ABS over PLA (charge current + ambient heat); orient so layer lines
  don't split along the contact-force axis.

---

## 4. Wiring & the CC details

### 4a. Robot side (battery → tail pads)

```
 battery USB-C VBUS ──[ 5A fuse ]── tail VBUS pad
 battery USB-C GND  ─────────────── tail GND pad
 battery USB-C CC   ─────────────── tail CC pad   (shorter/recessed — see 4c)
```

- Use the USB-C **male breakout** (A3) plugged into the battery so VBUS/GND/CC are
  cleanly labeled; route a short pigtail to the tail pads.
- Fuse the **VBUS line at the battery end** of the pigtail.

### 4b. Dock side (brick → pogo)

```
 brick → USB-C cable → female breakout (A2):  VBUS → VBUS pogo(s)
                                              GND  → GND pogo(s)
                                              CC   → CC pogo
```

### 4c. The two CC details (verify on the bench, normal for DIY USB-C)

1. **Pick the right CC pin.** A USB-C plug has CC1 and CC2; only one carries the
   live CC for a given orientation. Wire **one** CC line through (source CC ↔ sink
   CC). **Do not tie CC1+CC2 together** — that confuses VCONN/Ra detection.
   Build it, and if it doesn't start charging, move the CC wire to the other CC
   pin on the breakout. (One of the two will enumerate.)
2. **Make CC break first on undock.** Recess/shorten the **CC pad** (or its pogo)
   so that as the robot pulls away, **CC disconnects a moment before the VBUS/GND
   pads.** The instant CC opens, the brick removes VBUS — so the power pads
   separate already de-energized → near-zero arc. (Same principle as a mains
   plug's ground pin, inverted: here CC is "break-first.")

---

## 5. Commissioning — bench first, then the robot

1. **Bench-prove the splice with the battery, by hand.** Wire the dock breakout
   to its pogos and the robot breakout to the tail pads. Plug the brick in, plug
   the robot breakout into the battery, and **touch the pogos to the pads by
   hand.** Confirm the battery starts charging (its own indicator / rising pack
   voltage). If nothing happens, swap the CC wire to the other CC pin (§4c).
2. **Confirm dead-until-mated.** With the brick powered but nothing seated, meter
   the dock pogos — should read **0 V**. With the robot breakout *unplugged from
   the battery*, bridge the tail VBUS/GND pads — confirm **no current flows**
   (battery won't source without a sink on CC).
3. **Confirm break-first.** Mate, confirm charging, then slowly separate and
   watch for arc/spark at the power pads — there should be essentially none if CC
   breaks first. Adjust pad/pin lengths if you see arcing.
4. **First drive-in — wheels off the ground** (per the repo safety rule). Back in
   slowly, confirm the pads seat against the hard stop and charging starts; pull
   out and confirm it stops cleanly.

Notes:
- **Charge while running or shut down.** Charging works with the robot powered on
  (the BMS handles simultaneous charge/discharge), but the Jetson + ROSMASTER
  draw reduces net charge rate. Shut the robot down on the dock for fastest/cool
  charging.
- At 60 W, budget ~1.5–2.5 h for a full charge of a 99–155 Wh pack from low.

---

## 6. Optional — make it truly hands-off (autonomy)

You already run AMCL + LIDAR + Nav2. To close the loop:

- Add an **ArUco marker / reflective tape** (C6) on the dock for a precise final
  approach.
- Publish **`/battery_state`** (the ROSMASTER reports pack voltage) and a
  **`/docked`** flag (a microswitch on the hard stop, or "pack voltage rising"),
  so a behavior tree can run **low-battery → navigate-to-dock → seat → charge →
  undock**. v1 can be manual park-and-charge; this is a separate software task.

How does the robot know it's full / when to undock? It can't read SoC over PD,
so watch its **own pack voltage via the ROSMASTER** — climbing = charging, ~full
and holding = undock. Time-based/scheduled also works.

---

## Appendix — fallback if USB-C charging is ever unavailable

If you ever need to charge a non-USB-C V-mount through its raw V-lock ± rails
instead, the dock must then supply **16.8 V CC-CV** itself (e.g., Mean Well
HLG-185H-16A trimmed to 16.8 V), with a **sense-then-energize relay** (dead pins
until a robot-side ID resistor is detected) and **robot-side reverse-polarity
P-FET + fuse + inrush NTC**, because that path puts raw voltage on the cells. The
USB-C path above avoids all of that by letting the battery be its own charger.
This is documented only as a contingency — the USB-C design is the recommended
build.

---

*Generated from a ce-ideate session, 2026-06-13. Design confirmed against the
user's working setup: a USB-C PD (MacBook) charger charges this battery directly,
so the dock only needs to present a PD source on back-in contacts. Bench-verify
the CC-pin selection (§4c) and dead-until-mated behavior (§5) before driving.*
