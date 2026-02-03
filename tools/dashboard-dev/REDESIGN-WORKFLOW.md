# Dashboard Redesign Workflow

This document explains how we collaborate on the Blackbox dashboard redesign using Claude Code, including branch conventions, iteration patterns, and design exploration methodology.

---

## How We Work

### The Collaboration Model

Design happens through rapid iteration cycles:

1. **You describe** what you want (visual direction, feel, specific changes)
2. **Claude implements** changes directly in `index.html`
3. **You review** in the browser (localhost:8080)
4. **Repeat** until it feels right

Each significant change gets its own git branch, creating a visual history of the design exploration.

### Session Setup

```bash
# Start the local dev server
cd tools/dashboard-dev
python3 -m http.server 8080

# Open in browser
open http://localhost:8080
```

The dashboard runs a 30-second canyon drive simulation loop automatically - no hardware needed.

---

## Branch Naming Convention

Every design iteration creates a new branch with this format:

```
yyyy-mm-dd--HH-MM_description-of-change
```

| Component | Format | Example |
|-----------|--------|---------|
| Date | `yyyy-mm-dd` | `2026-01-19` |
| Separator | `--` | `--` |
| Time | `HH-MM` | `15-31` |
| Separator | `_` | `_` |
| Description | `kebab-case` | `apple-neumorphism-redesign` |

**Full example:** `2026-01-19--15-31_center-bottom-box-unify-1px-borders`

**Why this format?**
- Branches sort chronologically in git
- Easy to see the evolution of design decisions
- Descriptive names capture intent for future reference

---

## Design Exploration History

### Session: January 19, 2026

Starting from the original dark "Tron" aesthetic, we explored:

| Branch | Direction | Key Changes |
|--------|-----------|-------------|
| `..15-31_center-bottom-box-unify-1px-borders` | Polish | Centered metrics, unified 1px borders |
| `..15-35_unified-global-color-system` | System | Global `--hue` variable, cyan/amber/hotpink states |
| `..15-40_faster-color-fades-bottom-box-text` | Feel | Faster transitions, improved text legibility |
| `..15-45_legibility-remove-borders-fix-perspective` | Breathing | Removed outer borders, more whitespace |
| `..15-48_reorder-top-box-elements` | Hierarchy | Peak label → peak value → speed → maneuver |
| `..15-51_clean-ui-flat-gmeter` | Simplify | Removed background grids, flat G-meter |
| `..15-53_gmeter-true-perspective` | Depth | Added perspective tilt to G-meter |
| `..15-55_proper-perspective-math` | Correct | Mathematically correct 1-point perspective |
| `..16-05_apple-neumorphism-redesign` | New direction | Complete Apple-style neumorphic redesign |

### Design Evolution

```
Original "Tron" aesthetic
    │
    ├── Polish phase (borders, colors, transitions)
    │
    ├── Simplification phase (remove chrome, add air)
    │
    ├── Perspective experiments (G-meter depth)
    │
    └── Neumorphism pivot (completely new visual language)
```

---

## Working Style

### Giving Feedback

Be direct. Short phrases work best:

| Good | Less Good |
|------|-----------|
| "make it pop more" | "I think maybe we could try making it a bit more vibrant?" |
| "too busy, simplify" | "There's perhaps a few too many visual elements competing" |
| "your perspective sucks" | "The perspective doesn't quite feel right to me" |
| "kill the grid" | "Could we possibly remove the grid lines?" |

Claude responds better to clear, confident direction.

### Requesting Changes

**Single changes:**
> "center all the numbers"

**Multiple changes:**
> "center numbers, make borders 1px, remove the glow"

**Mood/direction:**
> "make it feel more Apple"
> "this is too dark, lighten it up"
> "needs more breathing room"

**Reference-based:**
> "like the neumorphism trend but cleaner"
> "flat design but with subtle depth"

### When to Branch

Create a new branch when:
- Trying a new visual direction
- Making structural layout changes
- Changing the design system (colors, typography)
- Experimenting with something you might want to revert

Don't need a new branch for:
- Tiny tweaks between reviews
- Bug fixes
- Typo corrections

---

## Technical Notes

### File Structure

```
tools/dashboard-dev/
├── index.html          # The entire dashboard (HTML + CSS + JS)
├── README.md           # Dev tool documentation
└── REDESIGN-WORKFLOW.md  # This file
```

Everything lives in one file (`index.html`) for simplicity. The production dashboard will be embedded in Rust firmware as a string literal, so single-file is mandatory.

### What Claude Can Do

| Capability | Examples |
|------------|----------|
| CSS changes | Colors, spacing, typography, shadows, borders |
| Layout changes | Flexbox, grid, positioning, responsive sizing |
| JavaScript | Animation timing, smoothing, state logic |
| Canvas rendering | G-meter visualization, trails, grids |
| Git operations | Branch, commit, push (after auth setup) |

### What Requires Your Input

- Subjective aesthetic judgment ("does this feel right?")
- Hardware testing (this is a simulator)
- GitHub authentication (one-time setup)
- Production deployment

---

## Neumorphism Design System (Current)

The latest iteration uses Apple-style neumorphism:

### Core Principles

1. **Soft depth** - Elements appear to extrude from or press into the surface
2. **Light background** - `#e8ecef` (soft grey)
3. **Dual shadows** - Dark shadow + light highlight creates 3D effect
4. **Rounded corners** - 24px standard, 50px for pills
5. **Generous whitespace** - 24px gaps, large padding

### Shadow System

```css
/* Raised element (speed card) */
.neu-raised {
    box-shadow:
        8px 8px 16px #c5c9cc,    /* Dark shadow (bottom-right) */
        -8px -8px 16px #ffffff;  /* Light highlight (top-left) */
}

/* Pressed/inset element (G-meter) */
.neu-inset {
    box-shadow:
        inset 6px 6px 12px #c5c9cc,
        inset -6px -6px 12px #ffffff;
}

/* Button with gradient */
.neu-btn {
    background: linear-gradient(145deg, #f0f4f7, #dce0e3);
    box-shadow:
        4px 4px 8px #c5c9cc,
        -4px -4px 8px #ffffff;
}
```

### Typography

- **Font:** `-apple-system, BlinkMacSystemFont, 'SF Pro Display', system-ui, sans-serif`
- **Speed:** 140px, weight 200 (ultralight)
- **Labels:** 10-14px, weight 600, letter-spacing 2px
- **Colors:** Primary `#2c3e50`, secondary `#7f8c8d`, muted `#a0aab0`

### Accent Color

Single accent: `#3498db` (soft blue) - used for:
- Status indicators
- G-meter dot and trail
- Maneuver text
- Active states

---

## Returning to Previous Designs

Every branch is preserved. To revisit:

```bash
# See all design branches
git branch --list '2026-*'

# Check out a previous design
git checkout 2026-01-19--15-51_clean-ui-flat-gmeter

# Compare two designs
git diff 2026-01-19--15-31_center.. 2026-01-19--16-05_apple..
```

---

## Future Sessions

When starting a new design session:

1. **Check current state:** `git status` and `git branch`
2. **Start server:** `python3 -m http.server 8080`
3. **Open browser:** `http://localhost:8080`
4. **Tell Claude** what direction to explore
5. **Iterate** with direct feedback

The branch history serves as your design journal - you can always trace back how you got here.
