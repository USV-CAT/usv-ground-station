# The oil-skimmer USV: concept

A small catamaran that recovers spilled oil from the water surface, driven from
a Steam Deck. This note covers the general idea: the boat, how a clean-up is
run, and how the simulator models each part so the game behaves like the real
job.

## Why a catamaran

A catamaran has two hulls with open water between them. That gap is exactly
where a skimmer wants to be:

- oil is funnelled between the bows;
- the skimmer sits in calm water in the middle of the boat, sheltered by the
  hulls;
- the boat steers and stops by running its two motors at different speeds, so
  it can slow to boom speed and pivot without a rudder.

## The payload

| Part | What it does | In the simulator |
|---|---|---|
| **Sweep arms** (short booms in a V from the bows) | Funnel a swath of the slick, a few times the gap between the hulls, into the skimmer | `skimmer.swath_m` = 2.4 m; drawn orange ahead of the boat when deployed |
| **Oleophilic brush or belt skimmer** between the hulls | Oil sticks to the brushes and water mostly does not; a scraper puts the oil into a sump | Recovered fluid is `oil_content` = 70 % oil |
| **Pump and tank** | Lift the sump to an on-board tank | Pump limit `max_rate_lpm` = 8 L/min; tank `tank_l` = 60 L; draws `power_a` = 4 A |
| **Oil-in-water fluorometer** under the hull | Tells you whether you are in the oil, and how much | `oil_ppm`, film thickness in µm, and the Bonn Agreement appearance code |
| **Echo sounder, water-quality sonde** | Depth, temperature, salinity, DO, pH… | Already part of the survey boat |

## How a clean-up runs

1. **Find the slick.** It shows on the chart as dark (thick) to grey-violet
   (sheen). The Oil page shows the film thickness under the boat and the
   *slick drift*: the current plus about 3 % of the wind, the standard rule
   for surface oil.
2. **Plan a sweep.** *Plan sweep on slick* fits a rectangle round the slick,
   aligned with its drift. It lays lanes one swath apart and sets a slow
   skimming speed on every lane except the run out to it. The pattern is
   planned relative to the **water**, as real sweeps are: the whole route
   drifts with the oil, so the lanes stay on the slick while you work.
3. **Deploy and go.** Use the **Skim** button (or **L3** on the Deck), then
   **A** to run the mission. Booms only hold oil slowly:
   - at about 0.7 kn, roughly 85 % of the oil met is kept;
   - above about 1.5 kn, oil is carried under the arms and efficiency
     collapses;
   - an alarm says so when you are going too fast.
4. **Watch the tank.** When it is full, stow the skimmer and come alongside
   the ground station (`GCS`). The tank pumps out at 12 L/min while you sit
   there with the skimmer stowed.
5. **Score.** *The spill* box totals what was released, what is still
   floating, what you recovered, what evaporated, and what washed **ashore**.
   Oil left long enough drifts onto the beach, which is the outcome to
   prevent.

## The model

- **Slick:** Lagrangian parcels, the method GNOME and OpenDrift use. Each one
  is moved by current + 3 % wind plus a random-walk spread
  (0.03 m²/s), loses a little to evaporation, and beaches where the chart
  says land.
- **Recovery** per step is the least of three quantities:
  - the oil inside the swath × efficiency(speed);
  - the pump limit;
  - the space left in the tank.
- **Power:** the pump's draw shows on the Power page and shortens the
  endurance forecast.

## Toward real hardware

- A real payload would report the same fields as `SkimmerState`: deployed,
  rate, tank level, oil ppm.
- The station would send `send_payload("skimmer", on)` over the link; with
  MAVLink that maps naturally onto a relay or servo command plus a few named
  values.
- Oil in water could come from an off-the-shelf hydrocarbon fluorometer.
  Tank level could come from a pressure or ultrasonic level sensor.
- An aerial or satellite slick outline, or the boat's own fluorometer track,
  would replace the simulated slick on the chart.
