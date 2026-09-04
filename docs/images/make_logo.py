"""Generate logo.svg and favicon.svg (run: python docs/images/make_logo.py docs/images).

A panda lying across a sim | real split: wireframe on the simulation side, solid
on the real side, a robot arm on each, and a dashed sim -> real arc between them.
Pure SVG, no dependencies; edit the coordinates here rather than the SVG.
"""
import pathlib, sys

W, H, MID = 640, 320, 320
OUT = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else pathlib.Path("docs/images")

def panda(style):
    """Panda shapes. style='solid' (real) or 'wire' (sim)."""
    if style == "solid":
        white = 'fill="#ffffff" stroke="#111827" stroke-width="3"'
        black = 'fill="#111827" stroke="#111827" stroke-width="3"'
        eye_w = 'fill="#ffffff"'; pupil = 'fill="#111827"'; hi = 'fill="#ffffff"'
        blush = 'fill="#fca5a5" opacity="0.7"'; mouth = 'stroke="#111827"'
    else:
        white = 'fill="#0b1220" fill-opacity="0.85" stroke="#7dd3fc" stroke-width="2.5"'
        black = 'fill="#0b1220" fill-opacity="0.85" stroke="#7dd3fc" stroke-width="2.5" stroke-dasharray="5 3"'
        eye_w = 'fill="#0b1220" stroke="#7dd3fc" stroke-width="2"'; pupil = 'fill="#7dd3fc"'; hi = 'fill="#e0f2fe"'
        blush = 'fill="none" stroke="#7dd3fc" stroke-width="1.5" opacity="0.7"'; mouth = 'stroke="#7dd3fc"'
    return f"""
    <!-- body -->
    <ellipse cx="320" cy="270" rx="122" ry="46" {white}/>
    <!-- forearms stretched forward, chin resting on them; one on each side of the seam -->
    <ellipse cx="258" cy="292" rx="54" ry="19" transform="rotate(-10 258 292)" {black}/>
    <ellipse cx="382" cy="292" rx="54" ry="19" transform="rotate(10 382 292)" {black}/>
    <!-- ears -->
    <circle cx="262" cy="148" r="27" {black}/>
    <circle cx="378" cy="148" r="27" {black}/>
    <!-- head -->
    <circle cx="320" cy="200" r="72" {white}/>
    <!-- eye patches -->
    <ellipse cx="290" cy="198" rx="21" ry="27" transform="rotate(-18 290 198)" {black}/>
    <ellipse cx="350" cy="198" rx="21" ry="27" transform="rotate(18 350 198)" {black}/>
    <!-- eyes -->
    <circle cx="293" cy="201" r="9" {eye_w}/>
    <circle cx="347" cy="201" r="9" {eye_w}/>
    <circle cx="294" cy="202" r="5.5" {pupil}/>
    <circle cx="346" cy="202" r="5.5" {pupil}/>
    <circle cx="296.5" cy="199.5" r="2" {hi}/>
    <circle cx="348.5" cy="199.5" r="2" {hi}/>
    <!-- nose + mouth + blush -->
    <ellipse cx="320" cy="224" rx="9" ry="6.5" {pupil if style=='wire' else 'fill="#111827"'}/>
    <path d="M 306 236 q 14 11 28 0" fill="none" {mouth} stroke-width="2.5" stroke-linecap="round"/>
    <circle cx="266" cy="226" r="8" {blush}/>
    <circle cx="374" cy="226" r="8" {blush}/>
    """

def robot_wire():
    s = 'fill="none" stroke="#7dd3fc" stroke-width="6" stroke-linecap="round" stroke-linejoin="round"'
    j = 'fill="#0b1220" stroke="#7dd3fc" stroke-width="3"'
    return f"""
    <g filter="url(#glow)">
      <rect x="72" y="212" width="76" height="18" rx="4" {s}/>
      <path d="M 110 212 L 96 142 L 152 96 L 194 112" {s}/>
      <path d="M 194 112 l 14 -8 M 194 112 l 14 8" fill="none" stroke="#7dd3fc" stroke-width="5" stroke-linecap="round"/>
      <circle cx="110" cy="212" r="9" {j}/><circle cx="96" cy="142" r="9" {j}/><circle cx="152" cy="96" r="9" {j}/>
    </g>
    """

def robot_solid():
    link = 'fill="none" stroke="#f9fafb" stroke-width="16" stroke-linecap="round" stroke-linejoin="round"'
    edge = 'fill="none" stroke="#6b7280" stroke-width="20" stroke-linecap="round" stroke-linejoin="round"'
    j = 'fill="#111827"'
    return f"""
    <g>
      <rect x="490" y="210" width="80" height="20" rx="5" fill="#d1d5db" stroke="#6b7280" stroke-width="3"/>
      <path d="M 530 212 L 544 142 L 488 96 L 446 112" {edge}/>
      <path d="M 530 212 L 544 142 L 488 96 L 446 112" {link}/>
      <path d="M 446 112 l -14 -9 M 446 112 l -14 9" fill="none" stroke="#374151" stroke-width="6" stroke-linecap="round"/>
      <circle cx="530" cy="212" r="10" {j}/><circle cx="544" cy="142" r="10" {j}/><circle cx="488" cy="96" r="10" {j}/>
    </g>
    """

grid = "".join(f'<line x1="{x}" y1="0" x2="{x}" y2="{H}"/>' for x in range(0, MID + 1, 32)) + \
       "".join(f'<line x1="0" y1="{y}" x2="{MID}" y2="{y}"/>' for y in range(0, H + 1, 32))

svg = f"""<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" role="img" aria-label="FrankaTwin: a panda lying across a simulation | reality split, with a wireframe robot arm on the sim side and a real one on the other">
  <defs>
    <linearGradient id="simbg" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#0f172a"/><stop offset="1" stop-color="#1e3a8a"/></linearGradient>
    <linearGradient id="realbg" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#fffbeb"/><stop offset="1" stop-color="#fde68a"/></linearGradient>
    <linearGradient id="seam" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#7dd3fc"/><stop offset="1" stop-color="#f59e0b"/></linearGradient>
    <linearGradient id="arc" x1="0" y1="0" x2="1" y2="0"><stop offset="0" stop-color="#7dd3fc"/><stop offset="1" stop-color="#f59e0b"/></linearGradient>
    <clipPath id="clipL"><rect x="0" y="0" width="{MID}" height="{H}"/></clipPath>
    <clipPath id="clipR"><rect x="{MID}" y="0" width="{W-MID}" height="{H}"/></clipPath>
    <filter id="glow" x="-20%" y="-20%" width="140%" height="140%"><feGaussianBlur stdDeviation="1.5" result="b"/><feMerge><feMergeNode in="b"/><feMergeNode in="SourceGraphic"/></feMerge></filter>
  </defs>

  <!-- backgrounds -->
  <rect x="0" y="0" width="{MID}" height="{H}" fill="url(#simbg)"/>
  <g stroke="#38bdf8" stroke-opacity="0.16" stroke-width="1">{grid}</g>
  <rect x="{MID}" y="0" width="{W-MID}" height="{H}" fill="url(#realbg)"/>
  <rect x="{MID}" y="230" width="{W-MID}" height="{H-230}" fill="#e7d8c3"/>
  <line x1="{MID}" y1="230" x2="{W}" y2="230" stroke="#c9b28f" stroke-width="2"/>
  <line x1="0" y1="230" x2="{MID}" y2="230" stroke="#7dd3fc" stroke-opacity="0.5" stroke-width="2"/>

  <!-- labels -->
  <text x="18" y="34" font-family="ui-monospace, SFMono-Regular, Menlo, monospace" font-size="15" font-weight="700" letter-spacing="3" fill="#7dd3fc">SIM</text>
  <text x="{W-18}" y="34" text-anchor="end" font-family="ui-monospace, SFMono-Regular, Menlo, monospace" font-size="15" font-weight="700" letter-spacing="3" fill="#b45309">REAL</text>

  <!-- robots -->
  {robot_wire()}
  {robot_solid()}

  <!-- sim -> real transfer arc -->
  <path d="M 200 104 Q 320 34 440 104" fill="none" stroke="url(#arc)" stroke-width="3.5" stroke-dasharray="7 6" stroke-linecap="round"/>
  <path d="M 440 104 l -12 -8 M 440 104 l -13 5" fill="none" stroke="#f59e0b" stroke-width="3.5" stroke-linecap="round"/>

  <!-- panda: wireframe on the sim side, solid on the real side -->
  <g clip-path="url(#clipL)">{panda('wire')}</g>
  <g clip-path="url(#clipR)">{panda('solid')}</g>

  <!-- seam -->
  <rect x="{MID-2}" y="0" width="4" height="{H}" fill="url(#seam)" opacity="0.9"/>
</svg>
"""
OUT.mkdir(parents=True, exist_ok=True)
(OUT / "logo.svg").write_text(svg)

# favicon: head only, on a rounded split background
fav = f"""<svg xmlns="http://www.w3.org/2000/svg" viewBox="244 118 152 158" width="64" height="64">
  <defs>
    <clipPath id="fl"><rect x="244" y="118" width="76" height="158"/></clipPath>
    <clipPath id="fr"><rect x="320" y="118" width="76" height="158"/></clipPath>
  </defs>
  <rect x="244" y="118" width="76" height="158" fill="#0f172a"/>
  <rect x="320" y="118" width="76" height="158" fill="#fde68a"/>
  <g clip-path="url(#fl)">{panda('wire')}</g>
  <g clip-path="url(#fr)">{panda('solid')}</g>
</svg>
"""
(OUT / "favicon.svg").write_text(fav)
print("wrote", OUT / "logo.svg", OUT / "favicon.svg")
