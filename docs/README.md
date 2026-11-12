# Hardware Guide — GitHub Pages

This directory contains the static HTML/CSS/JS hardware guide for the
[TechTed89-5G-Cellular-Drone-DIY-Build](https://github.com/techted89/TechTed89-5G-Cellular-Drone-DIY-Build)
project. It is designed to be deployed directly to **GitHub Pages** with no build step.

## Files

```
docs/
├── index.html       # Main hardware guide page
├── css/
│   └── styles.css   # All styling (responsive, dark-header hero, tables)
├── js/
│   └── main.js      # Scroll-spy + smooth-scroll navigation
└── images/          # Place your build photos here (see below)
```

## Deploy to GitHub Pages

1. Push this repo to GitHub (main branch).
2. In the repository **Settings → Pages**:
   - Build and deployment → Source: **Deploy from a branch**
   - Branch: `main` (or `gh-pages`)
   - Folder: `/docs`
   - Click **Save**.
3. Your site will be available at:
   `https://techted89.github.io/TechTed89-5G-Cellular-Drone-DIY-Build/`

## Adding Images

Place photos in `docs/images/` and reference them relative to the page root, e.g.:

```html
<img src="images/frame-build.jpg" alt="Carbon frame with FC and SBC">
```

Placeholder `.placeholder` items are already in the gallery section; replace them with real `<img>` tags.

## Content Covered

- **SBC / Companion Computers** — Raspberry Pi 5, Jetson Orin Nano Super, Odroid M1S, Orange Pi 5 (pro/cons, price, drone fit)
- **Flight Controllers** — Pixhawk 6C, Pixhawk 6X, Cube Orange+, Kakute H7, custom FC+SBC PCB design
- **5G / Cellular Modems** — RM520N-GL (eSIM), RM500Q-GL, SIM7600XX; SIM vs eSIM comparison; Metro by T-Mobile $20/mo plan
- **LoRaWAN** — RAK3172, RAK4631 backup telemetry
- **HATs / Add-ons** — Waveshare 5G HAT, Sixfab, Holybro RPi CM baseboard, Arduino
- **Power Train** — LiPo batteries, PDB, ESCs, brushless motors
- **Build Plans** — Entry (~$450), Recommended BVLOS (~$900), Pro AI (~$1,600)
- **Custom Build Design** — Step-by-step wiring overview + system diagram
- **Resources** — Vendor links, firmware, carriers, cameras

## Browser Support

Modern browsers only. Uses CSS Grid, Flexbox, `scroll-behavior`, and vanilla JS ES6+. No frameworks, no bundler.

## License

Same as the parent repository (see root `LICENSE`).
