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

# (file stem, printed width, caption). Order follows the paper.
FIGURES = [
    ('fig_trochoidal_design', FULL,
     'Designed against flown position for the trochoidal law, over the first '
     '20 s of each flight, at four of the eight gain rungs. Grey is the same '
     'configuration in the noiseless simulator, started from where the drones '
     'actually were when the algorithm engaged; colour is VICON. At k·τ 0.56 '
     'and 0.84 the aircraft sits on the designed pattern. At 1.12 a ripple '
     'appears on top of it and does not decay, and at 3.92 the pattern is '
     'barely visible beneath it. The loops in the left-hand panels have a '
     'period of about 1.1 s at every rung, set by the loop delay rather than '
     'by the gain.'),
    ('fig_noise_trochoidal_box', COL,
     'Deviation from the designed trochoidal pattern against injected '
     'position noise, at β = 2 where the gain ladder is quiet, so the '
     'exact-state assumption is probed on its own. Box is the middle half of '
     'the flight, whiskers the 5th to 95th percentile. Every other parameter '
     'is identical across the four flights.'),
    ('fig_coverage_design', FULL,
     'Designed against flown position for Voronoi coverage at three gain '
     'settings, 20 s from the middle of each flight. The label in each right '
     'panel is the median distance between the two lines over that window.'),
    ('fig_coverage_benchmarks', FULL,
     'Coverage under injected position noise. Top row: how the aircraft flew. '
     'Deviation and oscillation are computed from the recorded position, '
     'which carries the injected noise, so each bar is shown against a '
     'sensor-only null — the control flight re-measured through that rung\'s '
     'noise. Tilt comes from the motion-capture pose stream, which the '
     'injection never touches. Bottom row: the three quantities the coverage '
     'result is stated in. All three span a few percent of their own mean '
     'across a ladder that moves the aircraft by a factor of five.'),
    ('fig_noise_coverage_box', COL,
     'Deviation from the designed coverage configuration against injected '
     'position noise, with the sensor-only null marked on each rung.'),
    ('fig_flocking_design', FULL,
     'Designed against flown position for Olfati-Saber flocking at three '
     'time-rescalings, 20 s from the middle of each flight. The rescaling '
     'leaves the lattice and the path unchanged and moves only the rate at '
     'which the law acts, so k·τ is the only quantity that differs between '
     'rows.'),
    ('fig_noise_flocking_box', COL,
     'Deviation from the designed flocking lattice against injected position '
     'noise. The control sits above the 2 mm rung; both were flown twice and '
     'reproduce to 0.3 cm, so the ordering is a real property of the ladder '
     'rather than a bad flight.'),
    ('fig_noise_kuramoto_box', COL,
     'Deviation from the designed ring for the Kuramoto law against injected '
     'position noise. Every rung sits on its own sensor-only null, so the '
     'injection accounts for the whole reading and the flight itself does not '
     'degrade — unlike the trochoidal and coverage ladders, which move by a '
     'factor of five under the same lever.'),
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
