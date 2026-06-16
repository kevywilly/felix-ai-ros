# Felix charging dock — dimensioned mechanical spec

Companion to `charging-dock.md` (electrical/BOM). Two printed parts:
**(A)** a frame-mounted **L-bracket** on the robot that carries the 3 charge
pads, and **(B)** the **dock** the robot backs into. Reference model:
`3d/dock_reference.stl` (import into Fusion as a canvas/reference body — it is a
*massing* reference, not print-final; model the final part natively).

All dimensions in **mm**. Verified against the measured robot.

## Datums & coordinate frame

- **X** = lateral (right–left), **centerline X = 0**
- **Y** = fore/aft, **+Y into the robot**; **rear contact plane Y = 0** (where
  dock pins meet robot pads). Robot body at Y > 0, dock at Y < 0.
- **Z** = up, **floor Z = 0**.

## Robot rear — measured givens

| Feature | Value |
|---|---|
| Wheels | Ø96 × 38 wide, **track 265** c-c → inner faces **227 apart** |
| Wheel axle height | 48 |
| Battery rear face | **107 wide × 72 tall**, centered (X=0), **flat / stick-on-able** |
| Battery vertical span | **Z 12 → 84** (centered on axle, 48) |
| Back frame | 192 wide × 43 tall, **Z 34.5 → 77.5** (shifted up 8) |
| USB-C port | **left edge** of battery, plug exits **laterally (out the left)** |
| Barrel jack (12 V → ROSMASTER) | left side, through a frame hole — share this channel for the pigtail |
| **Pad target** | **X = 0, Z = 48** (center of rear face) |

---

## Part A — robot-side L-bracket (carries the pads)

Frame-referenced so the battery stays swappable (only the USB-C pigtail plugs
into it) and the pads sit in the exact same place every dock.

```
 TOP VIEW                         right motor-mount plate
   battery (107)                  ┌──┐
  ┌───────────────┐              ╔╧╧═╗  ← (1) mount leg bolts here
  │   (seated)    │══════════════╣   ║
  │               │  (2) arm     ╚═══╝
  └───────────────┘   across back
   ▲USB-C(left)   ▣▣▣ (3) pad platform @ center, pads face -Y (toward dock)
        └─pigtail along arm─┘
```

| # | Element | Spec |
|---|---|---|
| 1 | **Mount leg** | Flat plate against the **right motor-mount plate**. Bolt with **2× existing holes** (or M3 self-tappers / heat-set). ~4 thick. Match the plate's hole pitch — measure in Fusion. |
| 2 | **Arm** | Spans right mount → centerline, ~**75–95 long**. Section **≥6 thick × ~20 tall**, with a **stiffening rib** (cantilever carries pin pressure). Let the arm's inner face rest lightly on the battery back so docking force routes arm→battery→frame. |
| 3 | **Pad platform** | At center, **~100 wide × 25 tall**, pad surface at **Y = 0** (proud of the battery face by the arm offset ~5). Holds the 3 pads. **Chamfer the vertical side edges 45°** so the dock funnel can also help center it. |

**Constraints baked in**
- **Don't trap the battery** — arm spans the rear *plane* only; nothing wraps
  the top/bottom, so the battery still comes out its normal way after the USB-C
  is unplugged. *(Confirm removal direction; if it pulls rearward, make the arm
  removable / split.)*
- **Clear the left I/O** — arm comes from the right, stops at center, nowhere
  near the USB-C plug or barrel jack.
- **Pigtail channel** — a routed groove/clips along the arm from the left-side
  USB-C to the pad platform; tie alongside the existing barrel-jack cable.

### Pads (on the platform, lanes at X = −33 / 0 / +33, all centered Z = 48)

| Lane | X | Pad size (W × H) | Proud of platform | Role |
|---|---|---|---|---|
| VBUS | −33 | 25 × 18 | 0 (flush, full proud) | power + |
| GND | 0 | 25 × 18 | 0 | power − |
| CC | +33 | 14 × 18 | **recessed 1.0** | handshake — **break-first** |

- **CC recessed 1 mm** → on undock it loses contact first; the PD source kills
  VBUS before the power pads part (arc-free). See `charging-dock.md` §4c.
- Pads: **tin- or gold-plated copper**, 0.5–1 thick, bonded to the platform,
  soldered to the pigtail leads on the back side.

---

## Part B — the dock (reference: `dock_reference.stl`)

Tapered pocket the robot reverses into between its rear wheels. Footprint
**162 wide × 115 deep × 86 tall**.

| Element | Spec | Notes |
|---|---|---|
| **Funnel mouth** | **150 wide** (inner) at entry (Y=0) | 227 wheel gap → **38 mm/side** clearance to wheels |
| **Funnel throat** | **113 wide** (inner) at seat (Y=−35) | 107 battery → **3 mm/side** → self-centers lateral + yaw |
| **Funnel depth** | **35** (Y 0 → −35) | half-angle **≈28°**; kept **< battery depth (40)** so the walls stay beside the battery and clear the motor plates |
| **Funnel walls** | **6 thick**, height **Z 5 → 86** | catch the battery's vertical side edges (Z 12–84) |
| **Base plate** | **5 thick**, Z 0–5 | battery bottom (Z12) clears it by 7 |
| **Back wall** | Y −40 → −35, half-width 66, Z 5–86 | carries the floating pogo block |
| **Seat plane** | **Y = −35** | robot pad plane meets pins here |
| **Hard-stop bumpers** | 2 posts at X ±50, Z 48, **proud to Y = −33** | robot face seats on these, NOT the pins |
| **Brick shelf** | Y −40 → −110, back lip 30 tall | PD brick sits here behind the wall |

### Floating pogo block (in the back wall)

- Carries **5 pins** matched to the pads:
  - **VBUS** X=−33: 2 pins at Z=42 & 54 (doubled, parallel)
  - **GND** X=0: 2 pins at Z=42 & 54 (doubled, parallel)
  - **CC** X=+33: 1 pin at Z=48
- Pins: **gold 2–3 mm pogo, ≥5 A**, free length **proud 4** of the back wall,
  **compressed ~1.5** at seat (bumpers set the stop).
- **Block floats ±5 in X and Z**, spring-centered (T-slot + 2 light springs),
  to absorb the offset between battery-center and frame-center.

### Why this tolerates a sloppy back-in

Gross centering = funnel on the **battery** (107 in 113). Fine tolerance =
**wide pads (25)** + **floating pins (±5)**. Vertical is free (robot rides the
floor → pads & pins both fixed at Z=48). So the robot only needs to put its rear
into the **150 mm mouth** between the wheels — Nav2 back-in easily clears that.

---

## Tolerances & print notes

- **Print PETG or ABS** (charge current + ambient heat); PLA only for fit tests.
- Funnel walls & base **≥4 perimeters / solid** — they take docking force.
- **Contact-clearance fits:** throat is battery +3/side; if your battery seats
  loose, tighten throat toward 109–111.
- **45° chamfer every funnel entry edge** (printed; also avoids catch points).
- Add a **stiffening rib** on the L-bracket arm; print the arm flat so layer
  lines don't split along the pin-force axis.
- Heat-set **M3 inserts** for the pogo-block mount and the L-bracket fasteners.

## Using the reference STL in Fusion

`3d/dock_reference.stl` is in the same datum as above (mouth at Y=0, seat at
Y=−35, floor Z=0, centerline X=0). Drop it next to `RearEnd.stl`, align floors
and centerlines, slide the dock in −Y until the bumpers meet the battery face,
and check: (1) mouth clears both wheels, (2) battery noses to the throat,
(3) pins land on the pad lanes at Z=48. Then model the production part natively.

*Generated 2026-06-13 from the measured robot + `RearEnd.stl`. Reference STL
built by `3d/build_dock_stl.py`; layout drawing `3d/dock_layout.py`. Edit either
script's PARAMS and re-run to regenerate.*
