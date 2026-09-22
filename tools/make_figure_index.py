#!/usr/bin/env python3
"""Build one page that shows every ladder figure, instead of 20-odd PNGs.

    python3 tools/make_figure_index.py                  # -> docs/figures/index.html
    python3 tools/make_figure_index.py --embed          # also a standalone copy

The plain version references the PNGs next to it, so it stays small and updates
whenever a plot script is re-run. `--embed` inlines every image as a data URI
into index_standalone.html, which is the one to send to someone else.

Captions are kept here rather than in the plot scripts because they are prose
about what the figure MEANS, and that belongs where a reader can edit it without
touching analysis code. Numbers in the stat strips are read from each ladder's
summary.json, so they cannot drift from the figures.
"""

import argparse
import base64
import html
import json
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIG = os.path.join(ROOT, 'docs', 'figures')

LADDERS = [
    dict(key='trochoidal_ladder', name='Trochoidal', accent='#c2571a',
         blurb='Eight rungs of the same pattern, β from 1 to 14. The rung where '
               'the oscillation starts is the whole result.'),
    dict(key='coverage_ladder', name='Coverage', accent='#2a6db5',
         blurb='Three speeds of the same coverage law chasing a moving hotspot. '
               'Coverage was the quiet algorithm, so this asks what it takes to '
               'make it ring.'),
    dict(key='flocking_ladder', name='Flocking', accent='#1f8a63',
         blurb='The same flock, rescaled in time so k·τ drops from 1.3 to 0.4 '
               'without changing the lattice or the path.'),
]

# The two sweeps that hold k*tau FIXED and vary how hard the task is. They are
# not rungs of the delay ladder, so they get their own tab rather than being
# mixed into one of the three algorithm sections.
LEVER_SECTIONS = [
    dict(key='coverage_hotspot', name='Hotspot speed', accent='#c2571a',
         blurb='Coverage at three hotspot speeds, gains untouched, all flown '
               '22 Sep. k·τ is 0.67 in every run, so anything that changes here '
               'is the task getting harder, not the delay margin moving.'),
    dict(key='flocking_sense', name='Sense range', accent='#1f8a63',
         blurb='Flocking at two sense ranges, gains and spacing untouched, both '
               'flown 22 Sep. Shrinking the range takes neighbours out of the '
               'graph without changing the spacing the law is asked to hold.'),
]

CAPTIONS = {
    # --- levers (22 Sep) ----------------------------------------------------
    'coverage_hotspot/1_hotspot_speed.png': (
        'Hotspot speed: oscillation, lag and cost',
        'Three hotspot speeds, same gains, so k·τ is 0.67 in all three. The '
        'oscillation barely moves. The fleet flies about a second behind the '
        'hotspot at every speed, which is a bigger and bigger part of an orbit '
        'as the hotspot speeds up. The coverage cost rises with it, and '
        'hardware stays within 1% of simulation of the same file.'),
    'coverage_hotspot/3_design_vs_actual.png': (
        'Hotspot speed: distance from the designed position',
        'Top: where the law wanted the real drone against where it went, 25 s '
        'mid-flight. Bottom: the same distance at every row of the whole '
        'flight, so a run that is usually fine and occasionally bad cannot hide '
        'behind its median. 4 cm at 0.3 rad/s, 13 cm at 0.9, with the 95th '
        'percentile reaching 46 cm.'),
    'flocking_sense/1_sense_range.png': (
        'Sense range: spacing and oscillation',
        'Left: the distance the flock actually settled on between neighbouring '
        'drones, against the 0.70 m the law was told to hold. It settles short '
        'of the target and shorter still as the sense range shrinks — and '
        'simulation does the same thing, so this is the law, not the hardware. '
        'Right: the ripple on the real drone.'),
    'flocking_sense/3_design_vs_actual.png': (
        'Sense range: distance from the designed position',
        'Where the law wanted the real drone against where it went, and the '
        'distribution of that distance over the whole flight. The drone tracks '
        'its own design more closely at the shorter sense range, 6 cm against '
        '5 cm median, with a tighter spread.'),
    # --- trochoidal (15-16 Sep) --------------------------------------------
    'trochoidal_ladder/1_summary.png': (
        'Wobble, tilt and clipping across the ladder',
        'One row per rung, filled markers are the real drones. The step between '
        'β=3 and β=6 is where the command starts saturating and the wobble '
        'jumps by an order of magnitude.'),
    'trochoidal_ladder/2_pattern_size.png': (
        'Pattern size per lap, against the design',
        'The trochoid should hold its size. Above the threshold it does not, '
        'and the deviation is not a slow drift but a per-lap scaling.'),
    'trochoidal_ladder/3_shake.png': (
        'The wobble on its own',
        'Position band-passed to 0.6–1.5 Hz, so the designed pattern is removed '
        'and only the ripple is left. Loops, not noise.'),
    'trochoidal_ladder/4_delay.png': (
        'Loop delay, and the 4τ test',
        'Cross-correlation of the command against the acceleration achieved. '
        'The peak is τ; the measured ripple period sits on the period = 4τ line.'),
    'trochoidal_ladder/5_tilt.png': (
        'Bank angle demanded vs measured',
        'Tilt inferred from the path matches what VICON saw to 1–2°, which is '
        'what makes tilt usable as a proxy on older logs.'),
    'trochoidal_ladder/6_commanded_vs_actual.png': (
        'Commanded setpoint vs where the drone was',
        'The drone tracks its setpoint closely and late. The setpoint itself is '
        'what loops — worth being precise about, because it is not tracking error.'),
    'trochoidal_ladder/7_commanded_vs_actual_middle.png': (
        'The same, mid-flight',
        'A middle slice, away from the start transient and the landing.'),
    'trochoidal_ladder/8_design_vs_actual.png': (
        'Designed pattern vs flown pattern',
        'The design is simulated from the real start positions, not the config '
        'marks — the drones sat on each other\'s marks on 15 Sep.'),
    'trochoidal_ladder/9_design_vs_actual_middle.png': (
        'The same, mid-flight',
        'Middle window, same reference.'),
    'trochoidal_ladder/10_bursts_r3.png': (
        'β=3: the wobble comes in bursts',
        'Below the threshold the ringing decays, but it is re-excited — bursts '
        'that are not simultaneous across drones.'),
    # --- generic, applies to coverage and flocking -------------------------
    '*/1_summary.png': (
        'Wobble, the promised property, and the 4τ test',
        'Left: ripple per drone against the designed k·τ, filled = real. '
        'Middle: the property the theorem promises, hardware against the same '
        'config in simulation. Right: measured ripple period against 4 × the '
        'measured delay.'),
    '*/2_expected_vs_actual.png': (
        'Design vs actual',
        'Top: where the law wanted the real drone (its own config in '
        'simulation) against where it went, over 25 s mid-flight. Bottom: the '
        'distance between them.'),
    '*/3_promise.png': (
        'The promised property over time',
        'One panel per rung, hardware against simulation of that same file. A '
        'gap here is gap B — the physical layer — because simulation has already '
        'absorbed every modelling choice.'),
    '*/4_wobble.png': (
        'The wobble itself',
        'Position band-passed to 0.6–1.5 Hz. Closed loops mean a sustained '
        'oscillation; a scribble means noise.'),
    '*/5_delay.png': (
        'Loop delay per rung',
        'Left: the correlation curves, peak = τ. Right: real drones against the '
        'simulated ones, which share the software path but have no radio, motors '
        'or tilt — the difference is the physical half of the delay.'),
    '*/6_command.png': (
        'What the law asked the hardware for',
        'Saturation, bank angle and speed per rung. Clipping is the thing that '
        'turns a growing oscillation into a fixed-size one.'),
}


def _num(key, rung, field, scale=1.0, fmt='{:.0f}'):
    """One number from a ladder's summary.json, so captions cannot drift."""
    path = os.path.join(FIG, key, 'summary.json')
    if not os.path.exists(path):
        return '—'
    d = json.load(open(path)).get(rung, {})
    if field in d:
        return fmt.format(d[field] * scale)
    for dr in d.get('drones', {}).values():
        if dr.get('real') and field in dr:
            return fmt.format(dr[field] * scale)
    return '—'


def key_cards():
    """The three or four figures that carry the finding, with what they show."""
    g = lambda k, r, f, sc=1.0: _num(k, r, f, sc)
    return [
        ('key/1_threshold.png', 'The result, in one figure',
         'Eleven flights, three algorithms, three unrelated papers, plotted against '
         'a single number: the gain each law applies to a drone\'s own velocity, '
         'times the 0.28 s it takes that velocity to become thrust. Below '
         'k·τ ≈ 0.7 nothing oscillates. Above ≈ 1.0 everything does — and when it '
         'does, the period is 4τ regardless of which paper the law came from.'),
        ('coverage_ladder/2_expected_vs_actual.png',
         'Coverage: the quiet algorithm, made to ring on purpose',
         f'The same law at three speeds. At k·τ 0.34 and 0.67 the drone flies '
         f'where the law wanted it — {g("coverage_ladder", "c1", "design_gap")} and '
         f'{g("coverage_ladder", "c2", "design_gap")} cm away. At k·τ 1.01 it misses by '
         f'{g("coverage_ladder", "c3", "design_gap")} cm and covers the room in loops. '
         f'Coverage was the robust control condition; this is the first time the '
         f'project produced a failure rather than found one.'),
        ('flocking_ladder/2_expected_vs_actual.png',
         'Flocking: the same fix, on a different paper\'s law',
         f'Slowing the clock — position gains ×c², velocity gains ×c, same '
         f'lattice, same path — walks the drone from '
         f'{g("flocking_ladder", "s1", "design_gap")} cm off the designed position to '
         f'{g("flocking_ladder", "s2", "design_gap")} cm at half speed and '
         f'{g("flocking_ladder", "s3", "design_gap")} cm at a third, and the settled '
         f'lattice error from +41% against simulation to −1% to −0.1%. The gap to '
         f'simulation did not shrink; it closed.'),
        ('trochoidal_ladder/8_design_vs_actual.png',
         'Trochoidal: where the ladder started',
         'The original finding, kept here for comparison: the designed pattern '
         'against what flew, across β = 1 to 14. The design is simulated from '
         'the real start positions, because the drones sat on each other\'s '
         'marks on 15 Sep.'),
    ]


def _natural(name):
    """10_bursts sorts after 9_design, not after 1_summary."""
    head = name.split('_', 1)[0]
    return (int(head) if head.isdigit() else 999, name)


def caption(key):
    if key in CAPTIONS:
        return CAPTIONS[key]
    generic = '*/' + key.split('/', 1)[1]
    return CAPTIONS.get(generic, (key.split('/')[-1], ''))


def stat_strip(ladder_dir):
    """A compact table of the numbers behind the figures, if the plots wrote one."""
    path = os.path.join(ladder_dir, 'summary.json')
    if not os.path.exists(path):
        return ''
    data = json.load(open(path))

    def _ktau(item):
        reals = [d for d in item[1].get('drones', {}).values() if d.get('real')]
        return reals[0]['k_tau'] if reals else 9e9

    rows = []
    # Closest to what the theory asks for at the top, furthest at the bottom --
    # the same order the figures are drawn in.
    for rung, r in sorted(data.items(), key=_ktau):
        reals = [d for d in r.get('drones', {}).values() if d.get('real')]
        virts = [d for d in r.get('drones', {}).values() if not d.get('real')]
        if not reals:
            continue
        d = reals[0]
        gap = ''
        if r.get('promise_settled') and r.get('promise_sim_settled'):
            gap = f"{r['promise_settled'] / r['promise_sim_settled'] - 1:+.0%}"
        rows.append(
            f"<tr><td class='rung'>{html.escape(rung)}</td>"
            f"<td>{d['k']:.2f}</td><td>{d['k_tau']:.2f}</td>"
            f"<td>{d['lag']:.2f} s</td>"
            f"<td>{d['wobble'] * 100:.1f} cm</td>"
            f"<td>{(sum(v['wobble'] for v in virts) / len(virts) * 100):.1f} cm</td>"
            if virts else
            f"<tr><td class='rung'>{html.escape(rung)}</td>"
            f"<td>{d['k']:.2f}</td><td>{d['k_tau']:.2f}</td>"
            f"<td>{d['lag']:.2f} s</td><td>{d['wobble'] * 100:.1f} cm</td><td>—</td>")
        rows[-1] += (f"<td>{d['period']:.2f} s</td>" if d.get('period')
                     else '<td>—</td>')
        rows[-1] += f"<td>{d['clip']:.0%}</td><td>{gap or '—'}</td></tr>"
    if not rows:
        return ''
    return (
        "<table class='stats'><thead><tr>"
        "<th>rung</th><th>k</th><th>k·τ</th><th>τ measured</th>"
        "<th>wobble, real</th><th>wobble, sim'd</th><th>period</th>"
        "<th>clipped</th><th>vs sim</th>"
        "</tr></thead><tbody>" + ''.join(rows) + "</tbody></table>")


def img_src(path, embed):
    if not embed:
        return html.escape(os.path.relpath(path, FIG))
    with open(path, 'rb') as fh:
        return 'data:image/png;base64,' + base64.b64encode(fh.read()).decode()


CSS = """
:root{
  --ground:#f7f6f3; --card:#fffffe; --ink:#16181c; --ink-2:#4a4f57;
  --ink-3:#7c828c; --line:#e3e1db; --shadow:0 1px 2px rgba(20,18,14,.06),0 8px 24px rgba(20,18,14,.06);
}
@media (prefers-color-scheme:dark){ :root:not([data-theme="light"]){
  --ground:#14161a; --card:#1c1f25; --ink:#f2f3f5; --ink-2:#b9bec7;
  --ink-3:#848b96; --line:#2b3039; --shadow:0 1px 2px rgba(0,0,0,.4),0 8px 24px rgba(0,0,0,.35);
}}
:root[data-theme="dark"]{
  --ground:#14161a; --card:#1c1f25; --ink:#f2f3f5; --ink-2:#b9bec7;
  --ink-3:#848b96; --line:#2b3039; --shadow:0 1px 2px rgba(0,0,0,.4),0 8px 24px rgba(0,0,0,.35);
}
*{box-sizing:border-box}
body{margin:0;background:var(--ground);color:var(--ink);
  font:15px/1.55 ui-sans-serif,-apple-system,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;}
header.top{position:sticky;top:0;z-index:20;background:color-mix(in srgb,var(--ground) 88%,transparent);
  backdrop-filter:blur(8px);border-bottom:1px solid var(--line);padding:14px 28px;
  display:flex;gap:18px;align-items:baseline;flex-wrap:wrap}
header.top h1{font-size:16px;margin:0;letter-spacing:-.01em}
header.top .sub{color:var(--ink-3);font-size:13px}
nav{margin-left:auto;display:flex;gap:6px;flex-wrap:wrap}
nav button{font:inherit;font-size:13px;padding:5px 12px;border-radius:999px;cursor:pointer;
  border:1px solid var(--line);background:var(--card);color:var(--ink-2)}
nav button[aria-pressed="true"]{background:var(--ink);color:var(--ground);border-color:var(--ink)}
nav button:focus-visible{outline:2px solid var(--ink);outline-offset:2px}
main{padding:28px;max-width:1500px;margin:0 auto}
section{margin:0 0 46px}
section h2{font-size:22px;margin:0 0 4px;letter-spacing:-.02em;
  display:flex;align-items:center;gap:10px}
section h2::before{content:"";width:10px;height:10px;border-radius:3px;background:var(--accent)}
section .blurb{color:var(--ink-2);max-width:68ch;margin:0 0 16px}
table.stats{border-collapse:collapse;margin:0 0 22px;font-size:13px;
  font-variant-numeric:tabular-nums;background:var(--card);border:1px solid var(--line);
  border-radius:10px;overflow:hidden;box-shadow:var(--shadow)}
table.stats th{text-align:right;font-weight:600;color:var(--ink-3);font-size:11px;
  text-transform:uppercase;letter-spacing:.06em;padding:9px 13px;border-bottom:1px solid var(--line)}
table.stats td{text-align:right;padding:8px 13px;border-bottom:1px solid var(--line)}
table.stats tr:last-child td{border-bottom:none}
table.stats td.rung,table.stats th:first-child{text-align:left;font-weight:600;color:var(--ink)}
.grid{display:grid;gap:18px;grid-template-columns:repeat(auto-fill,minmax(430px,1fr))}
.keygrid{display:grid;gap:22px;grid-template-columns:repeat(auto-fit,minmax(560px,1fr))}
figure.big figcaption{padding:16px 18px 20px}
figure.big figcaption b{font-size:17px;letter-spacing:-.01em;margin-bottom:6px}
figure.big figcaption span{font-size:14px;line-height:1.5}
section.key h2::before{background:var(--ink)}
details{margin-top:6px}
details>summary{cursor:pointer;font-size:13px;color:var(--ink-2);padding:9px 14px;
  border:1px solid var(--line);border-radius:999px;display:inline-block;
  background:var(--card);list-style:none;user-select:none}
details>summary::-webkit-details-marker{display:none}
details>summary::before{content:"▸ ";color:var(--ink-3)}
details[open]>summary::before{content:"▾ "}
details>summary:hover{color:var(--ink)}
details>.grid{margin-top:18px}
figure{margin:0;background:var(--card);border:1px solid var(--line);border-radius:12px;
  overflow:hidden;box-shadow:var(--shadow);display:flex;flex-direction:column}
figure img{width:100%;display:block;background:#fff;cursor:zoom-in}
figcaption{padding:13px 15px 15px}
figcaption b{display:block;font-size:14px;margin-bottom:3px}
figcaption span{color:var(--ink-2);font-size:13px}
.empty{color:var(--ink-3);font-style:italic;padding:14px 0}
dialog{border:none;background:transparent;max-width:96vw;max-height:96vh;padding:0}
dialog::backdrop{background:rgba(10,10,12,.86)}
dialog img{max-width:96vw;max-height:88vh;background:#fff;border-radius:8px;display:block}
dialog .bar{display:flex;justify-content:space-between;align-items:center;gap:16px;
  color:#e9e9ea;font-size:13px;padding:10px 4px}
dialog button{font:inherit;background:transparent;border:1px solid #55585e;color:#e9e9ea;
  border-radius:8px;padding:4px 11px;cursor:pointer}
@media (prefers-reduced-motion:no-preference){figure{transition:transform .12s ease}
  figure:hover{transform:translateY(-1px)}}
"""

JS = """
const cards=[...document.querySelectorAll('figure')];
const dlg=document.querySelector('dialog'),dimg=dlg.querySelector('img'),
      dcap=dlg.querySelector('.cap');let idx=0;
function open_(i){idx=(i+cards.length)%cards.length;const f=cards[idx];
  dimg.src=f.querySelector('img').src;dcap.textContent=f.querySelector('b').textContent;
  if(!dlg.open)dlg.showModal();}
cards.forEach((f,i)=>f.querySelector('img').addEventListener('click',()=>open_(i)));
dlg.querySelector('.prev').onclick=()=>open_(idx-1);
dlg.querySelector('.next').onclick=()=>open_(idx+1);
dlg.addEventListener('click',e=>{if(e.target===dlg)dlg.close();});
addEventListener('keydown',e=>{if(!dlg.open)return;
  if(e.key==='ArrowRight')open_(idx+1);if(e.key==='ArrowLeft')open_(idx-1);});
const secs=[...document.querySelectorAll('section')];
document.querySelectorAll('nav button').forEach(b=>{
  b.onclick=()=>{const k=b.dataset.k;
    document.querySelectorAll('nav button').forEach(o=>o.setAttribute('aria-pressed',o===b));
    secs.forEach(s=>{s.hidden=(s.dataset.k!==k);});
    if(k!=='all'){const d=document.querySelector('section[data-k="'+k+'"] details');
      if(d)d.open=true;}
    scrollTo({top:0,behavior:'smooth'});};});
"""


def build(embed=False):
    parts = [f'<style>{CSS}</style>',
             '<header class="top"><h1>Multi-drone testbed — flight figures</h1>',
             '<span class="sub">what the delay does to three control laws</span>',
             '<nav><button data-k="all" aria-pressed="true">Findings</button>']
    for L in LADDERS + LEVER_SECTIONS:
        parts.append(f'<button data-k="{L["key"]}">{L["name"]}</button>')
    parts.append('</nav></header><main>')

    # ---- the findings, large and first -------------------------------------
    shown = set()
    cards = [(rel, t, n) for rel, t, n in key_cards()
             if os.path.exists(os.path.join(FIG, rel))]
    if cards:
        parts.append('<section class="key" data-k="all"><h2>The findings</h2>'
                     '<p class="blurb">Four figures carry the result. Everything '
                     'below them is supporting detail, folded away.</p>'
                     '<div class="keygrid">')
        for rel, title, note in cards:
            shown.add(rel)
            parts.append(
                f'<figure class="big"><img loading="lazy" '
                f'alt="{html.escape(title)}" '
                f'src="{img_src(os.path.join(FIG, rel), embed)}">'
                f'<figcaption><b>{html.escape(title)}</b>'
                f'<span>{html.escape(note)}</span></figcaption></figure>')
        parts.append('</div></section>')

    # ---- the two levers, open rather than collapsed -------------------------
    for L in LEVER_SECTIONS:
        d = os.path.join(FIG, L['key'])
        pngs = sorted((f for f in os.listdir(d) if f.endswith('.png')),
                      key=_natural) if os.path.isdir(d) else []
        parts.append(f'<section data-k="{L["key"]}" hidden '
                     f'style="--accent:{L["accent"]}">'
                     f'<h2>{L["name"]}</h2><p class="blurb">{L["blurb"]}</p>')
        if not pngs:
            parts.append('<p class="empty">No figures yet — run '
                         'tools/plot_levers.py.</p></section>')
            continue
        parts.append('<div class="keygrid">')
        for f in pngs:
            title, note = caption(f'{L["key"]}/{f}')
            shown.add(f'{L["key"]}/{f}')
            parts.append(
                f'<figure class="big"><img loading="lazy" '
                f'alt="{html.escape(title)}" '
                f'src="{img_src(os.path.join(d, f), embed)}">'
                f'<figcaption><b>{html.escape(title)}</b>'
                f'<span>{html.escape(note)}</span></figcaption></figure>')
        parts.append('</div></section>')

    # ---- per-algorithm detail, collapsed ------------------------------------
    total = len(shown)
    for L in LADDERS:
        d = os.path.join(FIG, L['key'])
        pngs = sorted((f for f in os.listdir(d) if f.endswith('.png')),
                      key=_natural) if os.path.isdir(d) else []
        rest = [f for f in pngs if f'{L["key"]}/{f}' not in shown]
        parts.append(f'<section data-k="{L["key"]}" hidden '
                     f'style="--accent:{L["accent"]}">'
                     f'<h2>{L["name"]}</h2><p class="blurb">{L["blurb"]}</p>')
        if not pngs:
            parts.append('<p class="empty">No figures yet — run the plot script '
                         'once the records are in logs/hw/.</p></section>')
            continue
        parts.append(stat_strip(d))
        parts.append(f'<details><summary>Supporting figures ({len(rest)})</summary>'
                     f'<div class="grid">')
        for f in rest:
            title, note = caption(f'{L["key"]}/{f}')
            parts.append(
                f'<figure><img loading="lazy" alt="{html.escape(title)}" '
                f'src="{img_src(os.path.join(d, f), embed)}">'
                f'<figcaption><b>{html.escape(title)}</b>'
                f'<span>{html.escape(note)}</span></figcaption></figure>')
            total += 1
        parts.append('</div></details></section>')

    parts.append('</main><dialog><img alt=""><div class="bar">'
                 '<button class="prev">&larr; Prev</button><span class="cap"></span>'
                 '<button class="next">Next &rarr;</button></div></dialog>')
    parts.append(f'<script>{JS}</script>')
    return ('<!doctype html><html lang="en"><head><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width,initial-scale=1">'
            '<title>Flight figures — multi-drone testbed</title></head><body>'
            + ''.join(parts) + '</body></html>'), total


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--embed', action='store_true',
                    help='also write index_standalone.html with images inlined')
    a = ap.parse_args()

    doc, n = build(embed=False)
    out = os.path.join(FIG, 'index.html')
    open(out, 'w').write(doc)
    print(f'{n} figures -> {out}')
    if a.embed:
        doc, n = build(embed=True)
        out = os.path.join(FIG, 'index_standalone.html')
        open(out, 'w').write(doc)
        print(f'{n} figures -> {out}  ({os.path.getsize(out)/1e6:.1f} MB, self-contained)')


if __name__ == '__main__':
    main()
