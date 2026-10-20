#!/usr/bin/env python3
"""One page showing the paper's figures as they will print, with their captions.

    python3 tools/make_paper_index.py            # -> docs/figures/paper_images.html
    python3 tools/make_paper_index.py --embed    # also a self-contained copy

Same look as index.html, different job. index.html is a working gallery for
every figure the project has produced; this is the shortlist that goes in the
report, rendered at its printed width so a column figure looks like a column
figure, with the caption text underneath ready to paste.

The figures carry no titles -- the caption below each one is the title.
"""
import argparse
import base64
import html
import os

from make_figure_index import CSS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIG = os.path.join(ROOT, 'docs', 'figures')
PAPER = os.path.join(FIG, 'paper')

COL, FULL = 3.5, 6.5     # the report's column and text widths, inches

# (file stem, printed width, caption). Order follows the paper, and inside
# each law it follows the order the supervisor's review asks for: the law
# working first (what it converged to, then the metric it is stated in), the
# sim-to-hardware comparison second, the sweeps last.
FIGURES = [
    # ---------------------------------------------------------- A. trochoidal
    ('fig_trochoidal_design', FULL,
     'Designed against flown position for the trochoidal law, over the first '
     '20 s of each flight, at four of the eight gain rungs. Grey is the same '
     'configuration in the noiseless simulator, started from where the drones '
     'actually were when the algorithm engaged; colour is VICON. At k\u00b7\u03c4 0.56 '
     'and 0.84 the aircraft sits on the designed pattern. At 1.12 a ripple '
     'appears on top of it and does not decay, and at 3.92 the pattern is '
     'barely visible beneath it. The loops in the left-hand panels have a '
     'period of about 1.1 s at every rung, set by the loop delay rather than '
     'by the gain.'),
    ('fig_noise_trochoidal_box', COL,
     'Deviation from the designed trochoidal pattern against injected '
     'position noise, at \u03b2 = 2 where the gain ladder is quiet, so the '
     'exact-state assumption is probed on its own. Box is the middle half of '
     'the flight, whiskers the 5th to 95th percentile. Every other parameter '
     'is identical across the four flights.'),
    # ------------------------------------------------------------ B. coverage
    ('fig_coverage_converge', FULL,
     'What the coverage law converged to, on the quiet rung (k\u00b7\u03c4 0.34). Left '
     'is the moment the algorithm engaged, right is 69 s later; both panels '
     'show the region Q, the Voronoi partition built from the flown '
     'positions, each agent (dot) joined to its own cell\u2019s centroid '
     '(cross), and the Gaussian density at one and two sigma. The law\u2019s '
     'fixed points are exactly the configurations where every agent sits on '
     'its own centroid, so the length of those joining lines is the '
     'convergence being claimed. The partition is built with the same '
     'half-plane clip the controller uses, so the drawn cells are the cells '
     'the cost was scored on.'),
    ('fig_coverage_promise', FULL,
     'The quantity the coverage result is stated in \u2014 the locational cost H '
     '\u2014 over the three gain settings, against the same configuration in the '
     'noiseless simulator. The density is a Gaussian hotspot orbiting once '
     'every 20.9 s, so H is a tracking cost rather than a settling one, and '
     'the slow undulation in both traces is one orbit of that hotspot. The '
     'first 5 s are cut: the fleet starts from its parked marks and that '
     'one-off fan-out is three times the size of everything after it. After '
     'the first 20 s the hardware median reads 0.147, 0.116 and 0.123 against '
     '0.164, 0.122 and 0.112 in simulation \u2014 within 0.017 on every rung. At '
     'k\u00b7\u03c4 1.01 the aircraft is visibly oscillating (next figure) and a fast '
     'ripple appears on H, but its median moves by 0.007. Each window ends '
     'where the aircraft begins its descent. The metric the '
     'theorem is written in barely registers a ladder that changes how the '
     'drones fly by a factor of five.'),
    ('fig_coverage_design', FULL,
     'Designed against flown position for Voronoi coverage at three gain '
     'settings, 20 s from the middle of each flight. The label in each right '
     'panel is the median distance between the two lines over that window.'),
    ('fig_coverage_benchmarks', FULL,
     'Coverage under injected position noise. Top row: how the aircraft flew. '
     'Deviation and oscillation are computed from the recorded position, '
     'which carries the injected noise, so each bar is shown against a '
     'sensor-only null \u2014 the control flight re-measured through that rung\u2019s '
     'noise. Tilt comes from the motion-capture pose stream, which the '
     'injection never touches. Bottom row: the three quantities the coverage '
     'result is stated in. All three span a few percent of their own mean '
     'across a ladder that moves the aircraft by a factor of five.'),
    ('fig_noise_coverage_box', COL,
     'Deviation from the designed coverage configuration against injected '
     'position noise. Box is the middle half of the flight, whiskers the 5th '
     'to 95th percentile, line the median. Median deviation runs 3.5, 6.4, '
     '5.8, 8.3 and 13.3 cm. Every parameter other than the noise level is '
     'identical across the five flights.'),
    # ------------------------------------------------------------ C. flocking
    ('fig_flocking_converge', FULL,
     'What the flocking law converged to, on the quiet rung (k\u00b7\u03c4 0.42). Left '
     'is the moment the algorithm engaged: the four agents sit on their '
     'parked marks, every pair further apart than the 0.84 m sense range, so '
     'no edge exists and there is no group. Right is 120 s later: a connected '
     'lattice at 0.59 m spacing that has also travelled to follow the moving '
     '\u03b3-agent. Both panels are drawn at the same scale. The edges are not '
     'scored good or bad here, because the \u03b1-lattice does not settle at the '
     'nominal spacing d = 0.70 m \u2014 the sigma-norm bump puts its equilibrium '
     'at about 0.84 d = 0.59 m, which is what the flight reached.'),
    ('fig_flocking_promise', FULL,
     'Lattice error \u2014 the RMS gap between every in-range pair and the nominal '
     'spacing \u2014 across the three time-rescalings, hardware against the same '
     'configuration in simulation. It settles at 0.1023, 0.1022 and 0.1049 m '
     'against 0.1084, 0.1026 and 0.1022 in simulation: flat across the whole '
     'ladder and within 0.6% of simulation on every rung, while the same '
     'flights move the aircraft from 5.3 to 17.9 cm off its designed '
     'position. The first 10 s are cut: the lattice is still forming there, so '
     'the error is averaged over fewer pairs and reads about a fifth of its '
     'settled value. The residual 10 cm is not a hardware error \u2014 the recorder '
     'scores against the nominal d = 0.70 m while the \u03b1-lattice equilibrium '
     'is at about 0.59 m, and the simulation carries the same offset. Each '
     'window ends where the aircraft begins its descent; left in, the landing '
     'adds a spike that reads as the law breaking down.'),
    ('fig_flocking_design', FULL,
     'Flown path against the design for Olfati-Saber flocking at three time '
     'rescalings. The rescaling leaves the lattice and the \u03b3-agent circle '
     'unchanged and moves only the rate at which the law acts, so each panel '
     'is the same arc of the same path and k\u00b7\u03c4 is the only quantity that '
     'differs. The windows are matched on arc rather than on time \u2014 3 rad of '
     'the reference circle, which at these rescalings takes 60, 40 and 20 s \u2014 '
     'because matching on time would have the slow rung draw a short stub and '
     'the fast one most of a lap. Grey is the same configuration in the '
     'noiseless simulator, started from where the drones actually were at '
     'engage. At k\u00b7\u03c4 0.42 and 0.62 the aircraft tracks the circle, 7 and '
     '9 cm off. At 1.25 a standing ripple appears on top of it \u2014 3.0 cm RMS '
     'in the 0.6\u20131.5 Hz band at a 1.16 s period, against 0.5 cm at the quiet '
     'rung \u2014 and the median miss trebles to 18 cm.'),
    ('fig_flocking_benchmarks', FULL,
     'Flocking under injected position noise. Top row: how the aircraft flew '
     '\u2014 deviation 5.2 \u2192 11.1 cm, oscillation 0.47 \u2192 1.24 cm, median tilt '
     '1.4 \u2192 2.1\u00b0. Bottom row: the properties the flocking result is stated '
     'in \u2014 cohesion, safety and connectivity. Lattice error spans 7% of its '
     'mean, the smallest pair gap 3%, and the graph is connected on 99.3% of '
     'ticks at every rung. The velocity-matching metric is left out: it reads '
     'the least-squares velocity, where the fit has already multiplied the '
     'injected position noise by about eleven, so its rise is the injection '
     'and not the flock.'),
    ('fig_noise_flocking_box', COL,
     'Deviation from the designed flocking lattice against injected position '
     'noise. Median deviation runs 5.1, 4.5, 8.2 and 9.1 cm. The control '
     'reads above the 2 mm rung; both of those conditions were flown twice '
     'and reproduce to within 0.3 cm, so the ordering is a property of the '
     'ladder rather than a bad flight.'),
    # ------------------------------------------------------------ D. kuramoto
    ('fig_kuramoto_converge', FULL,
     'The Kuramoto ring on the quiet rung (k\u00b7\u03c4 0.39), at engage and 74 s '
     'later. Unlike the other four laws the formation is also the starting '
     'condition \u2014 the drones take off onto marks that already lie on the '
     '0.65 m ring \u2014 so what the hardware tests is whether the ring is '
     'maintained, not whether it is reached, and the transient is '
     'correspondingly small: the order parameter runs 0.9994 at engage to '
     '1.0000 settled, the radial error off the ring 2.6 cm to 0.6 cm, and the '
     'angular spacing error 0.059 to 0.033 rad. Phase lock is therefore not '
     'evidence of much on this ladder \u2014 it was nearly perfect before the law '
     'did anything.'),
    ('fig_kuramoto_promise', FULL,
     'The order parameter R, the quantity the Kuramoto result is stated in, '
     'across the damping ladder. R reads 1.0000, 0.9987 and 0.9965 settled, '
     'against 1.0000 in simulation on every rung \u2014 within 0.4% of the ideal '
     'across a ladder that puts the aircraft into 21.5 cm of sustained '
     'oscillation at 29\u00b0 of bank with the acceleration clamp active on 83% of '
     'ticks. Kuramoto records carry no altitude channel, so each window here '
     'ends where the real aircraft stops moving while the simulated agents '
     'carry on; left in, the frozen agent drags R down and the drop reads as '
     'the law failing. A phase-only metric cannot see a position '
     'oscillation, which is the sharpest case of metric blindness in the '
     'set.'),
    ('fig_kuramoto_design', FULL,
     'Designed against flown position for the Kuramoto ring formation at '
     'three damping settings, 20 s from the middle of each flight. Only the '
     'own-velocity gain differs between rows; the ring the law asks for is '
     'identical. Across these rungs the smallest pair gap falls from 0.90 to '
     '0.68 m, the angular spacing error rises from 0.033 to 0.45 rad and the '
     'radial error off the ring from 0.6 to 14.0 cm \u2014 the ring deforms '
     'steadily \u2014 while R stays within 0.4% of 1 throughout.'),
    ('fig_kuramoto_benchmarks', FULL,
     'Kuramoto under injected position noise. Top row: how the aircraft flew. '
     'Tilt is absent because the Kuramoto recorder writes its own metric '
     'columns only and never the pose stream, so this law has no clean motion '
     'channel. Bottom row: the four properties the law promises. R is flat at '
     '1.0000 across the whole ladder, while the radial error off the ring '
     'spans 42% of its mean and the angular spacing error 53%. Phase lock '
     'holds; the geometry it is supposed to produce does not.'),
    ('fig_noise_kuramoto_box', COL,
     'Deviation from the designed ring for the Kuramoto law against injected '
     'position noise. Median deviation runs 12.0, 12.3, 11.8 and 12.1 cm \u2014 '
     'flat across the whole ladder, against a baseline gap of 12 cm. Under '
     'the same lever the trochoidal and coverage ladders move by a factor of '
     'four to five.'),
    # ------------------------------------------------- E. distance formation
    ('fig_distance_converge', FULL,
     'What the distance-based formation converged to, on the quiet rung '
     '(k\u00b7\u03c4 0.39). Left is the moment the algorithm engaged, right is 54 s '
     'later; the drawn graph is the octahedron\u2019s twelve constraint edges, '
     'orange where an edge is more than 30% off its target length. Three '
     'edges start out of tolerance and the hexagon is visibly skewed; by 54 s '
     'every edge is within tolerance. The edge-length error falls from 8.9 cm '
     'to 0.6 cm and the Lyapunov function W from 0.128 to 0.001 over the same '
     'window. Both panels are drawn at the same scale.'),
    ('fig_distance_promise', FULL,
     'Edge-length error, the quantity the distance-formation result is stated '
     'in, across the damping ladder. At k\u00b7\u03c4 0.39 the error settles at 0.6 cm '
     'against 0.1 cm in simulation; at 0.81 it holds at 10.7 cm, 27 times the '
     'simulated value, and at 1.20 at 10.0 cm, 17 times. Unlike coverage\u2019s H and '
     'Kuramoto\u2019s R, this metric is a direct function of the positions, so it '
     'registers the oscillation the aircraft is actually doing.'),
    ('fig_distance_design', FULL,
     'Designed against flown position for the distance-based formation at '
     'three damping settings, 20 s from the middle of each flight. The law '
     'holds a six-agent octahedron by inter-agent distances alone, with one '
     'real aircraft. At k\u00b7\u03c4 0.39 the drone sits on its station, 1 cm from '
     'where the law wanted it. At 0.81 it circles a 26 cm orbit around that '
     'station and at 1.20 a 40 cm one, at about 1.1 s per lap in both cases. '
     'Only the damping gain differs between rows; the commanded shape is '
     'identical.'),
    ('fig_distance_benchmarks', FULL,
     'Distance-based formation under injected position noise. Top row: how '
     'the aircraft flew. Bottom row: the three quantities the formation '
     'result is stated in \u2014 the edge-length error the theorem bounds, the '
     'shape error after a best rigid fit, and the Lyapunov function the '
     'convergence proof is built on. Unlike coverage, whose published '
     'quantities stay within a few percent of their mean across the same '
     'lever, all three move with the noise: the edge error by 54% of its '
     'mean, the shape error by 51%, and W by 133%. A shape defined by '
     'distances is a direct function of the positions the noise corrupts, so '
     'its metric cannot be blind to it.'),
    ('fig_noise_distance_box', COL,
     'Deviation from the designed octahedron against injected position noise, '
     'flown at k\u00b7\u03c4 0.39 where the gain ladder is quiet. Median deviation runs '
     '1.2, 2.4, 2.7 and 3.6 cm.'),
    # ------------------------------------------------------- F. all five laws
    ('fig_cross_noise', FULL,
     'All five control laws under the same lever. Deviation from the designed '
     'trajectory is the only quantity every law records, so it is the only '
     'axis the five can share \u2014 each law\u2019s own promise metric is in '
     'different units and measures a different property. Left, in '
     'centimetres: the baselines differ by a factor of nine, from 1.4 cm for '
     'the distance formation to 13.4 cm for Kuramoto, and that offset buries '
     'the slopes. Right, each law divided by its own 0 mm flight. Over 0 to '
     '10 mm the trochoidal law rises 5.7\u00d7, the distance formation 3.0\u00d7, '
     'coverage 2.9\u00d7 and flocking 2.1\u00d7, while Kuramoto ends at 0.99\u00d7 \u2014 flat. '
     'Kuramoto\u2019s flatness is not robustness: its 10 mm flight sits at 0.99 '
     'times its own sensor-only null, so the whole ladder is inside the gap '
     'the law already had. Coverage was the only law taken to 20 mm; the '
     'rungs are placed evenly rather than to scale. No null is drawn: the '
     'trochoidal ladder was flown before the null model existed, and a '
     'reference line present for four series and missing for the fifth would '
     'read as a difference between the laws rather than between the ladders.'),
]

EXTRA_CSS = """
.sheet{max-width:1180px;margin:0 auto;padding:0 18px 64px}
.pf{background:var(--card);border:1px solid var(--line);border-radius:12px;
    box-shadow:var(--shadow);margin:0 0 26px;padding:18px 18px 14px}
.pf .plate{background:#fff;border:1px solid var(--line);border-radius:6px;
    padding:10px;display:flex;justify-content:center}
.pf img{display:block;height:auto;image-rendering:auto}
.pf .meta{display:flex;gap:10px;align-items:baseline;margin:0 0 10px;flex-wrap:wrap}
.pf .num{font-weight:700;color:var(--ink)}
.pf .w{font-size:12px;color:var(--ink-3);font-variant-numeric:tabular-nums}
.pf .w code{background:var(--ground);padding:1px 6px;border-radius:4px}
.pf figcaption{margin:12px 2px 0;color:var(--ink-2);font-size:14.5px;line-height:1.55}
.pf figcaption b{color:var(--ink)}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]) .pf .plate{background:#f7f7f5}}
:root[data-theme="dark"] .pf .plate{background:#f7f7f5}
"""


def img_src(path, embed):
    if not embed:
        return html.escape(os.path.relpath(path, FIG))
    with open(path, 'rb') as fh:
        return 'data:image/png;base64,' + base64.b64encode(fh.read()).decode()


def build(embed=False):
    parts = [f'<style>{CSS}{EXTRA_CSS}</style>',
             '<header class="top"><h1>Paper figures</h1>',
             '<span class="sub">the shortlist that goes in the report, shown at '
             'the width it prints at — 3.5 in for a column, 6.5 in for full '
             'width. No figure carries a title: the caption is the title.'
             '</span></header><main class="sheet">']
    n = 0
    for stem, width, caption in FIGURES:
        png = os.path.join(PAPER, f'{stem}.png')
        if not os.path.exists(png):
            continue
        n += 1
        pdf = os.path.relpath(os.path.join(PAPER, f'{stem}.pdf'), FIG)
        parts.append(
            f'<figure class="pf"><div class="meta">'
            f'<span class="num">Fig. {n}</span>'
            f'<span class="w">{width:g} in &nbsp;·&nbsp; <code>{html.escape(stem)}.pdf</code>'
            f' &nbsp;·&nbsp; <a href="{html.escape(pdf)}">vector</a></span></div>'
            f'<div class="plate"><img loading="lazy" alt="Figure {n}" '
            f'style="width:{width / FULL * 100:.1f}%;max-width:{width * 150:.0f}px" '
            f'src="{img_src(png, embed)}"></div>'
            f'<figcaption><b>Fig. {n}.</b> {html.escape(caption)}</figcaption></figure>')
    parts.append('</main>')
    return ('<!doctype html><html lang="en"><head><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width,initial-scale=1">'
            '<title>Paper figures</title></head><body>'
            + ''.join(parts) + '</body></html>'), n


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--embed', action='store_true')
    a = ap.parse_args()
    doc, n = build(False)
    out = os.path.join(FIG, 'paper_images.html')
    open(out, 'w').write(doc)
    print(f'{n} figures -> {out}')
    if a.embed:
        doc, n = build(True)
        out = os.path.join(FIG, 'paper_images_standalone.html')
        open(out, 'w').write(doc)
        print(f'{n} figures -> {out}  ({os.path.getsize(out) / 1e6:.1f} MB)')


if __name__ == '__main__':
    main()
