/**
 * wind-inference.js
 * Shared wind inference module for M32 sailing tools.
 *
 * Exports (as globals):
 *   window.inferWind(tracks, wwLat, wwLon, lwLat, lwLon, gunTs, raceEndTs, polarGrid)
 *   window.computeDayWind(event)
 *   window.blendRaceAndDayWind(raceObj, dayObj)
 *
 * polarGrid is optional. If omitted, falls back to the embedded base M32 polar.
 * Hosts with a Firestore-updated polar should pass getPolarGrid() at call time.
 *
 * All math utilities are private to this module to avoid collisions with host globals.
 */
(function () {
  'use strict';

  // ─── MATH CONSTANTS ────────────────────────────────────────────────────────
  const D2R = Math.PI / 180;
  const RE  = 6371000;

  // ─── GEOMETRY UTILITIES (private) ──────────────────────────────────────────
  function hav(a, b, c, d) {
    const dL = (c - a) * D2R, dO = (d - b) * D2R;
    const x = Math.sin(dL / 2) ** 2 + Math.cos(a * D2R) * Math.cos(c * D2R) * Math.sin(dO / 2) ** 2;
    return RE * 2 * Math.asin(Math.sqrt(x));
  }
  function brg(a, b, c, d) {
    const dO = (d - b) * D2R;
    const x = Math.sin(dO) * Math.cos(c * D2R);
    const y = Math.cos(a * D2R) * Math.sin(c * D2R) - Math.sin(a * D2R) * Math.cos(c * D2R) * Math.cos(dO);
    return (Math.atan2(x, y) / D2R + 360) % 360;
  }
  function angDiff(a, b) { return ((a - b + 540) % 360) - 180; }

  function weightedCircMean(items) {
    if (!items || !items.length) return null;
    let sx = 0, sy = 0, sw = 0;
    for (const it of items) {
      const w = Math.max(0.01, it.weight || 1);
      sx += Math.cos(it.angle * D2R) * w;
      sy += Math.sin(it.angle * D2R) * w;
      sw += w;
    }
    if (!sw) return null;
    return (Math.atan2(sy / sw, sx / sw) / D2R + 360) % 360;
  }

  function circularBlend(a, wa, b, wb) {
    const total = wa + wb; if (!total) return a;
    const ax = Math.cos(a * D2R) * wa + Math.cos(b * D2R) * wb;
    const ay = Math.sin(a * D2R) * wa + Math.sin(b * D2R) * wb;
    return (Math.atan2(ay / total, ax / total) / D2R + 360) % 360;
  }

  // ─── BASE M32 POLAR TABLE ──────────────────────────────────────────────────
  // Source of truth for wind inference when no updated polar is provided.
  const M32_POLAR_BASE = {"6":{"50":9.2,"51":9.23,"52":9.26,"53":9.29,"54":9.32,"55":9.35,"56":9.38,"57":9.41,"58":9.44,"59":9.47,"60":9.5,"61":9.53,"62":9.56,"63":9.59,"64":9.62,"65":9.65,"66":9.68,"67":9.71,"68":9.74,"69":9.77,"70":9.8,"71":9.82,"72":9.84,"73":9.86,"74":9.88,"75":9.9,"76":9.91,"77":9.92,"78":9.93,"79":9.94,"80":9.95,"81":9.956,"82":9.962,"83":9.968,"84":9.974,"85":9.98,"86":9.984,"87":9.988,"88":9.992,"89":9.996,"90":10.0,"91":10.1,"92":10.2,"93":10.3,"94":10.4,"95":10.5,"96":10.6,"97":10.7,"98":10.8,"99":10.9,"100":11.0,"101":11.03,"102":11.06,"103":11.09,"104":11.12,"105":11.15,"106":11.17,"107":11.19,"108":11.21,"109":11.23,"110":11.25,"111":11.24,"112":11.23,"113":11.22,"114":11.21,"115":11.2,"116":11.16,"117":11.12,"118":11.08,"119":11.04,"120":11.0,"121":10.94,"122":10.88,"123":10.82,"124":10.76,"125":10.7,"126":10.64,"127":10.58,"128":10.52,"129":10.46,"130":10.4,"131":10.32,"132":10.24,"133":10.16,"134":10.08,"135":10.0,"136":9.98,"137":9.96,"138":9.94,"139":9.92,"140":9.9,"141":9.88,"142":9.86,"143":9.84,"144":9.82,"145":9.8,"146":9.78,"147":9.76,"148":9.74,"149":9.72,"150":9.7},"8":{"50":10.06,"51":10.078,"52":10.096,"53":10.114,"54":10.132,"55":10.15,"56":10.166,"57":10.182,"58":10.198,"59":10.214,"60":10.23,"61":10.254,"62":10.278,"63":10.302,"64":10.326,"65":10.35,"66":10.374,"67":10.398,"68":10.422,"69":10.446,"70":10.47,"71":10.57,"72":10.67,"73":10.77,"74":10.87,"75":10.97,"76":11.066,"77":11.162,"78":11.258,"79":11.354,"80":11.45,"81":11.514,"82":11.578,"83":11.642,"84":11.706,"85":11.77,"86":11.832,"87":11.894,"88":11.956,"89":12.018,"90":12.08,"91":12.176,"92":12.272,"93":12.368,"94":12.464,"95":12.56,"96":12.656,"97":12.752,"98":12.848,"99":12.944,"100":13.04,"101":13.04,"102":13.04,"103":13.04,"104":13.04,"105":13.04,"106":13.056,"107":13.072,"108":13.088,"109":13.104,"110":13.12,"111":13.118,"112":13.116,"113":13.114,"114":13.112,"115":13.11,"116":13.108,"117":13.106,"118":13.104,"119":13.102,"120":13.1,"121":13.052,"122":13.004,"123":12.956,"124":12.908,"125":12.86,"126":12.81,"127":12.76,"128":12.71,"129":12.66,"130":12.61,"131":12.57,"132":12.53,"133":12.49,"134":12.45,"135":12.41,"136":12.41,"137":12.41,"138":12.41,"139":12.41,"140":12.41,"141":12.422,"142":12.434,"143":12.446,"144":12.458,"145":12.47,"146":12.484,"147":12.498,"148":12.512,"149":12.526,"150":12.54},"10":{"50":10.93,"51":10.932,"52":10.934,"53":10.936,"54":10.938,"55":10.94,"56":10.944,"57":10.948,"58":10.952,"59":10.956,"60":10.96,"61":10.978,"62":10.996,"63":11.014,"64":11.032,"65":11.05,"66":11.066,"67":11.082,"68":11.098,"69":11.114,"70":11.13,"71":11.314,"72":11.498,"73":11.682,"74":11.866,"75":12.05,"76":12.23,"77":12.41,"78":12.59,"79":12.77,"80":12.95,"81":13.072,"82":13.194,"83":13.316,"84":13.438,"85":13.56,"86":13.68,"87":13.8,"88":13.92,"89":14.04,"90":14.16,"91":14.254,"92":14.348,"93":14.442,"94":14.536,"95":14.63,"96":14.72,"97":14.81,"98":14.9,"99":14.99,"100":15.08,"101":15.05,"102":15.02,"103":14.99,"104":14.96,"105":14.93,"106":14.94,"107":14.95,"108":14.96,"109":14.97,"110":14.98,"111":14.986,"112":14.992,"113":14.998,"114":15.004,"115":15.01,"116":15.046,"117":15.082,"118":15.118,"119":15.154,"120":15.19,"121":15.154,"122":15.118,"123":15.082,"124":15.046,"125":15.01,"126":14.97,"127":14.93,"128":14.89,"129":14.85,"130":14.81,"131":14.81,"132":14.81,"133":14.81,"134":14.81,"135":14.81,"136":14.83,"137":14.85,"138":14.87,"139":14.89,"140":14.91,"141":14.958,"142":15.006,"143":15.054,"144":15.102,"145":15.15,"146":15.196,"147":15.242,"148":15.288,"149":15.334,"150":15.38},"12":{"50":11.79,"51":11.78,"52":11.77,"53":11.76,"54":11.75,"55":11.74,"56":11.73,"57":11.72,"58":11.71,"59":11.7,"60":11.69,"61":11.702,"62":11.714,"63":11.726,"64":11.738,"65":11.75,"66":11.76,"67":11.77,"68":11.78,"69":11.79,"70":11.8,"71":12.064,"72":12.328,"73":12.592,"74":12.856,"75":13.12,"76":13.386,"77":13.652,"78":13.918,"79":14.184,"80":14.45,"81":14.63,"82":14.81,"83":14.99,"84":15.17,"85":15.35,"86":15.53,"87":15.71,"88":15.89,"89":16.07,"90":16.25,"91":16.338,"92":16.426,"93":16.514,"94":16.602,"95":16.69,"96":16.776,"97":16.862,"98":16.948,"99":17.034,"100":17.12,"101":17.062,"102":17.004,"103":16.946,"104":16.888,"105":16.83,"106":16.772,"107":16.714,"108":16.656,"109":16.598,"110":16.54,"111":16.616,"112":16.692,"113":16.768,"114":16.844,"115":16.92,"116":16.994,"117":17.068,"118":17.142,"119":17.216,"120":17.29,"121":17.264,"122":17.238,"123":17.212,"124":17.186,"125":17.16,"126":17.132,"127":17.104,"128":17.076,"129":17.048,"130":17.02,"131":17.06,"132":17.1,"133":17.14,"134":17.18,"135":17.22,"136":17.26,"137":17.3,"138":17.34,"139":17.38,"140":17.42,"141":17.5,"142":17.58,"143":17.66,"144":17.74,"145":17.82,"146":17.9,"147":17.98,"148":18.06,"149":18.14,"150":18.22},"14":{"50":12.19,"51":12.198,"52":12.206,"53":12.214,"54":12.222,"55":12.23,"56":12.242,"57":12.254,"58":12.266,"59":12.278,"60":12.29,"61":12.324,"62":12.358,"63":12.392,"64":12.426,"65":12.46,"66":12.496,"67":12.532,"68":12.568,"69":12.604,"70":12.64,"71":12.892,"72":13.144,"73":13.396,"74":13.648,"75":13.9,"76":14.244,"77":14.588,"78":14.932,"79":15.276,"80":15.62,"81":15.736,"82":15.852,"83":15.968,"84":16.084,"85":16.2,"86":16.392,"87":16.584,"88":16.776,"89":16.968,"90":17.16,"91":17.27,"92":17.38,"93":17.49,"94":17.6,"95":17.71,"96":17.812,"97":17.914,"98":18.016,"99":18.118,"100":18.22,"101":18.192,"102":18.164,"103":18.136,"104":18.108,"105":18.08,"106":18.05,"107":18.02,"108":17.99,"109":17.96,"110":17.93,"111":18.004,"112":18.078,"113":18.152,"114":18.226,"115":18.3,"116":18.37,"117":18.44,"118":18.51,"119":18.58,"120":18.65,"121":18.676,"122":18.702,"123":18.728,"124":18.754,"125":18.78,"126":18.732,"127":18.684,"128":18.636,"129":18.588,"130":18.54,"131":18.58,"132":18.62,"133":18.66,"134":18.7,"135":18.74,"136":18.776,"137":18.812,"138":18.848,"139":18.884,"140":18.92,"141":18.988,"142":19.056,"143":19.124,"144":19.192,"145":19.26,"146":19.324,"147":19.388,"148":19.452,"149":19.516,"150":19.58},"16":{"50":12.59,"51":12.616,"52":12.642,"53":12.668,"54":12.694,"55":12.72,"56":12.754,"57":12.788,"58":12.822,"59":12.856,"60":12.89,"61":12.946,"62":13.002,"63":13.058,"64":13.114,"65":13.17,"66":13.232,"67":13.294,"68":13.356,"69":13.418,"70":13.48,"71":13.722,"72":13.964,"73":14.206,"74":14.448,"75":14.69,"76":15.03,"77":15.37,"78":15.71,"79":16.05,"80":16.39,"81":16.522,"82":16.654,"83":16.786,"84":16.918,"85":17.05,"86":17.254,"87":17.458,"88":17.662,"89":17.866,"90":18.07,"91":18.2,"92":18.33,"93":18.46,"94":18.59,"95":18.72,"96":18.838,"97":18.956,"98":19.074,"99":19.192,"100":19.31,"101":19.314,"102":19.318,"103":19.322,"104":19.326,"105":19.33,"106":19.328,"107":19.326,"108":19.324,"109":19.322,"110":19.32,"111":19.392,"112":19.464,"113":19.536,"114":19.608,"115":19.68,"116":19.748,"117":19.816,"118":19.884,"119":19.952,"120":20.02,"121":20.096,"122":20.172,"123":20.248,"124":20.324,"125":20.4,"126":20.33,"127":20.26,"128":20.19,"129":20.12,"130":20.05,"131":20.09,"132":20.13,"133":20.17,"134":20.21,"135":20.25,"136":20.282,"137":20.314,"138":20.346,"139":20.378,"140":20.41,"141":20.466,"142":20.522,"143":20.578,"144":20.634,"145":20.69,"146":20.738,"147":20.786,"148":20.834,"149":20.882,"150":20.93},"18":{"50":13.0,"51":13.044,"52":13.088,"53":13.132,"54":13.176,"55":13.22,"56":13.276,"57":13.332,"58":13.388,"59":13.444,"60":13.5,"61":13.576,"62":13.652,"63":13.728,"64":13.804,"65":13.88,"66":13.968,"67":14.056,"68":14.144,"69":14.232,"70":14.32,"71":14.55,"72":14.78,"73":15.01,"74":15.24,"75":15.47,"76":15.808,"77":16.146,"78":16.484,"79":16.822,"80":17.16,"81":17.308,"82":17.456,"83":17.604,"84":17.752,"85":17.9,"86":18.116,"87":18.332,"88":18.548,"89":18.764,"90":18.98,"91":19.13,"92":19.28,"93":19.43,"94":19.58,"95":19.73,"96":19.866,"97":20.002,"98":20.138,"99":20.274,"100":20.41,"101":20.444,"102":20.478,"103":20.512,"104":20.546,"105":20.58,"106":20.604,"107":20.628,"108":20.652,"109":20.676,"110":20.7,"111":20.772,"112":20.844,"113":20.916,"114":20.988,"115":21.06,"116":21.124,"117":21.188,"118":21.252,"119":21.316,"120":21.38,"121":21.508,"122":21.636,"123":21.764,"124":21.892,"125":22.02,"126":21.93,"127":21.84,"128":21.75,"129":21.66,"130":21.57,"131":21.61,"132":21.65,"133":21.69,"134":21.73,"135":21.77,"136":21.798,"137":21.826,"138":21.854,"139":21.882,"140":21.91,"141":21.954,"142":21.998,"143":22.042,"144":22.086,"145":22.13,"146":22.162,"147":22.194,"148":22.226,"149":22.258,"150":22.29},"20":{"50":13.4,"51":13.462,"52":13.524,"53":13.586,"54":13.648,"55":13.71,"56":13.788,"57":13.866,"58":13.944,"59":14.022,"60":14.1,"61":14.198,"62":14.296,"63":14.394,"64":14.492,"65":14.59,"66":14.704,"67":14.818,"68":14.932,"69":15.046,"70":15.16,"71":15.38,"72":15.6,"73":15.82,"74":16.04,"75":16.26,"76":16.594,"77":16.928,"78":17.262,"79":17.596,"80":17.93,"81":18.094,"82":18.258,"83":18.422,"84":18.586,"85":18.75,"86":18.978,"87":19.206,"88":19.434,"89":19.662,"90":19.89,"91":20.062,"92":20.234,"93":20.406,"94":20.578,"95":20.75,"96":20.9,"97":21.05,"98":21.2,"99":21.35,"100":21.5,"101":21.566,"102":21.632,"103":21.698,"104":21.764,"105":21.83,"106":21.882,"107":21.934,"108":21.986,"109":22.038,"110":22.09,"111":22.16,"112":22.23,"113":22.3,"114":22.37,"115":22.44,"116":22.5,"117":22.56,"118":22.62,"119":22.68,"120":22.74,"121":22.92,"122":23.1,"123":23.28,"124":23.46,"125":23.64,"126":23.528,"127":23.416,"128":23.304,"129":23.192,"130":23.08,"131":23.12,"132":23.16,"133":23.2,"134":23.24,"135":23.28,"136":23.304,"137":23.328,"138":23.352,"139":23.376,"140":23.4,"141":23.432,"142":23.464,"143":23.496,"144":23.528,"145":23.56,"146":23.576,"147":23.592,"148":23.608,"149":23.624,"150":23.64},"22":{"50":13.8,"51":13.88,"52":13.96,"53":14.04,"54":14.12,"55":14.2,"56":14.3,"57":14.4,"58":14.5,"59":14.6,"60":14.7,"61":14.82,"62":14.94,"63":15.06,"64":15.18,"65":15.3,"66":15.44,"67":15.58,"68":15.72,"69":15.86,"70":16.0,"71":16.2,"72":16.4,"73":16.6,"74":16.8,"75":17.0,"76":17.26,"77":17.52,"78":17.78,"79":18.04,"80":18.3,"81":18.56,"82":18.82,"83":19.08,"84":19.34,"85":19.6,"86":19.84,"87":20.08,"88":20.32,"89":20.56,"90":20.8,"91":21.0,"92":21.2,"93":21.4,"94":21.6,"95":21.8,"96":21.96,"97":22.12,"98":22.28,"99":22.44,"100":22.6,"101":22.7,"102":22.8,"103":22.9,"104":23.0,"105":23.1,"106":23.18,"107":23.26,"108":23.34,"109":23.42,"110":23.5,"111":23.56,"112":23.62,"113":23.68,"114":23.74,"115":23.8,"116":23.86,"117":23.92,"118":23.98,"119":24.04,"120":24.1,"121":24.16,"122":24.22,"123":24.28,"124":24.34,"125":24.4,"126":24.44,"127":24.48,"128":24.52,"129":24.56,"130":24.6,"131":24.64,"132":24.68,"133":24.72,"134":24.76,"135":24.8,"136":24.82,"137":24.84,"138":24.86,"139":24.88,"140":24.9,"141":24.92,"142":24.94,"143":24.96,"144":24.98,"145":25.0,"146":25.0,"147":25.0,"148":25.0,"149":25.0,"150":25.0}};
  const M32_POLAR_TWS_LIST = [6, 8, 10, 12, 14, 16, 18, 20, 22];
  const M32_POLAR_TWA_MIN  = 50;
  const M32_POLAR_TWA_MAX  = 150;

  // ─── POLAR LOOKUP (private) ────────────────────────────────────────────────
  // Uses the passed-in grid (Firestore-updated) or falls back to base polar.
  function polarSpeedFn(twa, tws, grid) {
    const polar = grid || M32_POLAR_BASE;
    let t = Math.round(Math.abs(twa));
    if (t < M32_POLAR_TWA_MIN) t = M32_POLAR_TWA_MIN;
    if (t > M32_POLAR_TWA_MAX) t = M32_POLAR_TWA_MAX;
    const twsList = M32_POLAR_TWS_LIST;
    if (tws <= twsList[0]) return +(polar[String(twsList[0])][String(t)] || 0);
    if (tws >= twsList[twsList.length - 1]) return +(polar[String(twsList[twsList.length - 1])][String(t)] || 0);
    let lo = twsList[0], hi = twsList[twsList.length - 1];
    for (let i = 0; i < twsList.length - 1; i++) {
      if (tws >= twsList[i] && tws <= twsList[i + 1]) { lo = twsList[i]; hi = twsList[i + 1]; break; }
    }
    const f = (tws - lo) / (hi - lo);
    const slo = +(polar[String(lo)][String(t)] || 0);
    const shi = +(polar[String(hi)][String(t)] || 0);
    return slo + f * (shi - slo);
  }

  // Estimate TWS from observed SOG at a given TWA by scanning the polar
  function estimateTWSFn(observedSOG, twa, grid) {
    let bestTWS = 12, bestDiff = 999;
    for (const tws of M32_POLAR_TWS_LIST) {
      const diff = Math.abs(polarSpeedFn(twa, tws, grid) - observedSOG);
      if (diff < bestDiff) { bestDiff = diff; bestTWS = tws; }
    }
    return bestTWS;
  }

  // ─── VELOCITY VECTOR HELPERS (private) ────────────────────────────────────
  function vecFrom(sog, cog) {
    const r = cog * D2R;
    return { vx: sog * Math.sin(r), vy: sog * Math.cos(r) };
  }
  function vecMean(arr) {
    if (!arr.length) return null;
    let sx = 0, sy = 0, sw = 0;
    for (const a of arr) { const w = Math.max(0.01, a.weight || 1); sx += (a.vx || 0) * w; sy += (a.vy || 0) * w; sw += w; }
    if (!sw) return null;
    return { vx: sx / sw, vy: sy / sw, mag: Math.hypot(sx / sw, sy / sw) };
  }
  function vecBearing(v) { return ((Math.atan2(v.vx, v.vy) / D2R) + 360) % 360; }


  // ── Adaptive SOG floor ────────────────────────────────────────────────────
  // Scans all tracks for downwind-heading points in the inference window,
  // takes the median speed, back-calculates TWS from the polar, then sets
  // the upwind floor at 60% of polar upwind speed at that TWS.
  //
  //   ~5kt TWS  → floor ~5.5kts   (light air, e.g. Day 1 Worlds)
  //   ~12kt TWS → floor ~9.5kts   (moderate breeze)
  //   ~18kt TWS → floor ~11kts    (fresh)
  //
  // Hard floor 4.5kts — below that the boat is essentially stopped.
  function estimateSOGfloor(tracks, QUAL_BOATS, usableStart, usableEnd, courseAxis, polarGrid) {
    const downwindAxis = (courseAxis + 180) % 360;
    const dwSpeeds = [];

    for (const sail of QUAL_BOATS) {
      const t = tracks[sail];
      if (!t || t.length < 10) continue;
      for (const p of t) {
        if (p.ts < usableStart || p.ts > usableEnd) continue;
        const sog = (p.sog || 0) * 0.539957;
        if (sog < 2) continue;
        const hdg = p.hdg || 0;
        const relDW = Math.abs(((hdg - downwindAxis + 540) % 360) - 180);
        if (relDW < 65) dwSpeeds.push(sog);  // broad downwind cone
      }
    }

    if (dwSpeeds.length < 20) return 4.5;  // not enough data, use minimum

    dwSpeeds.sort((a, b) => a - b);
    const medianDW = dwSpeeds[Math.floor(dwSpeeds.length * 0.5)];

    // Back-calculate TWS: scan polar at VMG-downwind TWA (110°) for best match
    let bestTWS = 10, bestDiff = Infinity;
    for (const tws of M32_POLAR_TWS_LIST) {
      const diff = Math.abs(polarSpeedFn(110, tws, polarGrid) - medianDW);
      if (diff < bestDiff) { bestDiff = diff; bestTWS = tws; }
    }

    // Look up polar upwind speed at that TWS (VMG-upwind TWA ~55°)
    const polarUW = polarSpeedFn(55, bestTWS, polarGrid);

    // Floor = 60% of polar upwind, clamped to [4.5, 14]
    const floor = Math.max(4.5, Math.min(14, polarUW * 0.60));
    return +floor.toFixed(1);
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  inferWind — improved algorithm
  //
  //  Improvements over prior version:
  //  1. Iterative cone refinement: first pass ±65° off course axis for rough WF;
  //     second pass ±65° off rough WF for clean port/stbd separation.
  //  2. Polar-speed weighting: segments near polar target speed get higher weight,
  //     suppressing tacking/poor-trim data. Uses passed-in polarGrid if available.
  //  3. Tacking angle validation: port/stbd angular separation checked against
  //     expected M32 tacking angle (90–110°); confidence reduced if outside range.
  //  4. Adaptive SOG floor: derived from observed median downwind speed → back-
  //     calculated TWS → 60% of polar upwind speed at that TWS. Automatically
  //     lower in light air, higher in breeze. Hard floor 4.5kts.
  //  5. Tuned filters: hdgAgree 30° (was 24°), gentler downwind penalty (/60).
  // ═══════════════════════════════════════════════════════════════════════════
  function inferWind(tracks, wwgateLat, wwgateLon, lwgateLat, lwgateLon, gunTs, raceEndTs, polarGrid) {
    const QUAL_BOATS = Object.keys(tracks || {});
    const _gun = (gunTs != null ? gunTs : 0);

    let inferredEnd = raceEndTs;
    if (inferredEnd == null) {
      let mx = _gun + 720000;
      for (const sail of QUAL_BOATS) {
        const t = tracks[sail];
        if (t && t.length) mx = Math.max(mx, t[t.length - 1].ts || mx);
      }
      inferredEnd = mx;
    }

    const startTrimMs   = 120000;
    const capFromStartMs = 720000;
    const usableStart = _gun + startTrimMs;
    const usableEnd   = Math.max(usableStart + 90000, Math.min(inferredEnd, _gun + capFromStartMs));

    // Course axis from mark positions
    let courseAxis = 315;
    if (wwgateLat != null && lwgateLat != null) {
      courseAxis = brg(lwgateLat, lwgateLon, wwgateLat, wwgateLon);
    }

    // ── Adaptive SOG floor ────────────────────────────────────────────────
    const sogFloor = estimateSOGfloor(tracks, QUAL_BOATS, usableStart, usableEnd, courseAxis, polarGrid);

    // ── PASS 1: rough WF using ±65° cone off course axis ──────────────────
    const pass1 = collectSegments(tracks, QUAL_BOATS, usableStart, usableEnd, courseAxis, 65, polarGrid, sogFloor);
    let roughWF = courseAxis; // fallback if insufficient data
    {
      const { uwPort, uwStbd, dwPort, dwStbd } = pass1;
      const mUp = computeWFfromPair(uwPort, uwStbd, 3);
      const mDw = computeWFfromPair(dwPort, dwStbd, 3, true);
      if (mUp != null && mDw != null) {
        roughWF = circularBlend(mUp, (uwPort.length + uwStbd.length) * 1.2,
                                mDw, (dwPort.length + dwStbd.length) * 0.5);
      } else if (mUp != null) {
        roughWF = mUp;
      } else if (mDw != null) {
        roughWF = mDw;
      }
    }

    // ── PASS 2: refined segments using ±65° cone off roughWF ──────────────
    const pass2 = collectSegments(tracks, QUAL_BOATS, usableStart, usableEnd, roughWF, 65, polarGrid, sogFloor);
    const { uwPort, uwStbd, dwPort, dwStbd } = pass2;

    const upN = uwPort.length + uwStbd.length;
    const dwN = dwPort.length + dwStbd.length;

    const meanUwPort = vecMean(uwPort), meanUwStbd = vecMean(uwStbd);
    const meanDwPort = vecMean(dwPort), meanDwStbd = vecMean(dwStbd);

    let wfUp = null, wfDw = null, tackAngle = null, gybeAngle = null;

    if (meanUwPort && meanUwStbd && uwPort.length >= 4 && uwStbd.length >= 4) {
      const sum = { vx: meanUwPort.vx + meanUwStbd.vx, vy: meanUwPort.vy + meanUwStbd.vy };
      wfUp = vecBearing(sum);
      // Tacking angle validation
      const pm = weightedCircMean(uwPort.map(s => ({ angle: s.angle, weight: s.weight })));
      const sm = weightedCircMean(uwStbd.map(s => ({ angle: s.angle, weight: s.weight })));
      tackAngle = Math.abs(angDiff(pm, sm));
      if (tackAngle > 180) tackAngle = 360 - tackAngle;
      if (tackAngle > 90)  tackAngle = 180 - tackAngle;
    }
    if (meanDwPort && meanDwStbd && dwPort.length >= 4 && dwStbd.length >= 4) {
      const sum = { vx: meanDwPort.vx + meanDwStbd.vx, vy: meanDwPort.vy + meanDwStbd.vy };
      wfDw = (vecBearing(sum) + 180) % 360;
      const pm = weightedCircMean(dwPort.map(s => ({ angle: s.angle, weight: s.weight })));
      const sm = weightedCircMean(dwStbd.map(s => ({ angle: s.angle, weight: s.weight })));
      gybeAngle = Math.abs(angDiff(pm, sm));
      if (gybeAngle > 180) gybeAngle = 360 - gybeAngle;
      if (gybeAngle > 90)  gybeAngle = 180 - gybeAngle;
    }

    // ── BLEND up/down wind estimates ───────────────────────────────────────
    let wf = wfUp != null ? wfUp : (wfDw != null ? wfDw : courseAxis);
    let agreement = (wfUp != null && wfDw != null) ? Math.abs(angDiff(wfUp, wfDw)) : null;

    if (wfUp != null && wfDw != null) {
      const wUp = Math.max(1, upN) * 1.25;
      // Gentler downwind agreement penalty: /60 (was /45)
      const wDw = Math.max(1, dwN) * 0.55 * Math.max(0.3, 1 - (agreement || 0) / 60);
      wf = circularBlend(wfUp, wUp, wfDw, wDw);
    }

    // ── CONFIDENCE ─────────────────────────────────────────────────────────
    const stablePts = upN + dwN;

    // Spread of upwind samples around final wf
    const spreadFrom = (arr, wfRef) => {
      if (!arr.length) return 180;
      const vals = arr.map(s => Math.abs(angDiff(s.angle, wfRef))).sort((a, b) => a - b);
      return vals[Math.floor(vals.length * 0.7)] ?? 180;
    };
    const spread = Math.min(spreadFrom(uwPort, wf), spreadFrom(uwStbd, wf));

    let confidence = Math.min(0.98,
      0.16 +
      Math.min(0.40, stablePts / 140) +
      (wfUp != null ? 0.18 : 0) +
      (wfDw != null ? 0.06 : 0)
    );
    confidence *= Math.max(0.35, 1 - Math.max(0, (spread - 32)) / 40);
    if (agreement != null) confidence *= Math.max(0.45, 1 - agreement / 50);

    // Tacking angle penalty: expected 90–110° for M32
    if (tackAngle != null) {
      const tackDev = Math.max(0, Math.min(tackAngle, 180 - tackAngle) - 10); // deviation outside ±10° of expected ~100°
      // Mapped: deviation 0→no penalty, deviation 20°→0.85, deviation 40°→0.70
      confidence *= Math.max(0.65, 1 - tackDev / 80);
    }

    const fallback = wfUp == null && wfDw == null;
    if (fallback) confidence = 0.16;

    return {
      wf, upwindAxis: courseAxis, downwindAxis: (courseAxis + 180) % 360,
      wfUp, wfDw, agreement,
      port:  weightedCircMean(uwPort.map(s => ({ angle: s.angle, weight: s.weight }))),
      stbd:  weightedCircMean(uwStbd.map(s => ({ angle: s.angle, weight: s.weight }))),
      tack:  tackAngle || 90,
      gybe:  gybeAngle || null,
      n: pass2.perBoat.length, stableN: stablePts,
      portN: uwPort.length, stbdN: uwStbd.length,
      dwPortN: dwPort.length, dwStbdN: dwStbd.length,
      confidence: +confidence.toFixed(2),
      fallback,
      startTrimS: Math.round(startTrimMs / 1000),
      endTrimS: Math.max(0, Math.round((inferredEnd - usableEnd) / 1000)),
      usableStartTs: usableStart, usableEndTs: usableEnd,
      sogFloor, method: 'iterative-cone-polar-weighted-v2',
      samples: { uwPort, uwStbd, dwPort, dwStbd },
      means: { uwPort: meanUwPort, uwStbd: meanUwStbd, dwPort: meanDwPort, dwStbd: meanDwStbd }
    };
  }

  // ── Segment collection (shared by pass 1 and pass 2) ─────────────────────
  // halfCone: degrees either side of the axis to accept as upwind/downwind
  function collectSegments(tracks, QUAL_BOATS, usableStart, usableEnd, refAxis, halfCone, polarGrid, sogFloor) {
    const downwindAxis = (refAxis + 180) % 360;
    const uwPort = [], uwStbd = [], dwPort = [], dwStbd = [];
    const perBoat = [];

    // For polar weighting we need an estimate of TWS.
    // Use a mid-range default; the weighting is relative so exact value matters less than consistency.
    const twsEst = 12;

    for (const sail of QUAL_BOATS) {
      const t = tracks[sail];
      if (!t || t.length < 25) continue;
      let stableN = 0;

      for (let i = 3; i < t.length - 3; i++) {
        const p = t[i], a = t[i - 3], b = t[i + 3];
        if (!p || !a || !b) continue;
        if (p.ts < usableStart || p.ts > usableEnd) continue;

        const sog = (p.sog || 0) * 0.539957;
        if (sog < (sogFloor || 4.5) || sog > 35) continue;

        const dt = Math.max(0.001, ((b.ts || 0) - (a.ts || 0)) / 1000);
        const spanM = hav(a.lat, a.lon, b.lat, b.lon);
        if (dt < 6 || spanM < 20) continue;

        const cog     = brg(a.lat, a.lon, b.lat, b.lon);
        const cogPrev = brg(t[i - 3].lat, t[i - 3].lon, t[i - 1].lat, t[i - 1].lon);
        const cogNext = brg(t[i + 1].lat, t[i + 1].lon, t[i + 3].lat, t[i + 3].lon);
        const turn    = Math.abs(angDiff(cogPrev, cogNext));

        const hdgAgree = angDiff(cog, p.hdg || cog);
        const acc = Math.abs(((t[i + 1].sog || 0) - (t[i - 1].sog || 0)) * 0.539957);

        // Loosened hdgAgree from 24° → 30°
        if (turn > 16 || Math.abs(hdgAgree) > 30 || acc > 1.4) continue;

        let q = Math.max(0, 1 - turn / 18) *
                Math.max(0, 1 - Math.abs(hdgAgree) / 34) *
                Math.max(0, 1 - acc / 1.6) *
                Math.max(0, Math.min(1.2, spanM / 45));
        if (q < 0.28) continue;

        // ── Polar-speed weighting ──────────────────────────────────────────
        // Estimate TWA from cog vs refAxis so we can look up polar speed.
        const relToRef = Math.abs(angDiff(cog, refAxis));
        const twa = Math.min(relToRef, 180 - relToRef);  // 0–90° approximation
        const clampedTWA = Math.max(M32_POLAR_TWA_MIN, Math.min(M32_POLAR_TWA_MAX, twa < 50 ? 90 - twa : twa));
        const polarTarget = polarSpeedFn(clampedTWA, twsEst, polarGrid);
        if (polarTarget > 0) {
          // Reward segments near polar speed; cap boost at 1.5×
          q *= Math.min(1.5, sog / polarTarget);
        }
        if (q < 0.20) continue;  // re-check after polar weighting

        stableN++;
        const relUW = ((cog - refAxis + 540) % 360) - 180;
        const relDW = ((cog - downwindAxis + 540) % 360) - 180;
        const v = vecFrom(sog, cog);
        const row = { angle: cog, weight: q, sog, spanM, dt, vx: v.vx, vy: v.vy, boat: sail };

        if (Math.abs(relUW) <= halfCone) {
          if (relUW > 0) uwPort.push({ ...row, rel: relUW });
          else           uwStbd.push({ ...row, rel: relUW });
        } else if (Math.abs(relDW) <= halfCone) {
          if (relDW < 0) dwPort.push({ ...row, rel: relDW });
          else           dwStbd.push({ ...row, rel: relDW });
        }
      }
      if (stableN) perBoat.push({ sail, stableN });
    }
    return { uwPort, uwStbd, dwPort, dwStbd, perBoat };
  }

  // Helper: compute WF from a port/stbd pair. isDownwind flips 180°.
  function computeWFfromPair(portArr, stbdArr, minN, isDownwind) {
    if (portArr.length < minN || stbdArr.length < minN) return null;
    const mp = vecMean(portArr), ms = vecMean(stbdArr);
    if (!mp || !ms) return null;
    const sum = { vx: mp.vx + ms.vx, vy: mp.vy + ms.vy };
    const bearing = vecBearing(sum);
    return isDownwind ? (bearing + 180) % 360 : bearing;
  }


  // ═══════════════════════════════════════════════════════════════════════════
  //  computeDayWind — weighted consensus across all races in a day
  // ═══════════════════════════════════════════════════════════════════════════
  function computeDayWind(event) {
    const races = (event?._races || [])
      .filter(Boolean)
      .filter(rd => rd.WF_RACE_OBJ && rd.WF_RACE_OBJ.wf != null);
    if (!races.length) return null;

    const weighted = [];
    for (const rd of races) {
      const wObj = rd.WF_RACE_OBJ;
      const w = Math.max(0.15, (wObj.confidence || 0.2)) * Math.max(1, wObj.stableN || 10);
      weighted.push({ angle: wObj.wf, weight: w, raceNum: rd.raceNum });
    }
    const wf = weightedCircMean(weighted);
    const spreads = weighted.map(x => Math.abs(angDiff(x.angle, wf))).sort((a, b) => a - b);
    const spread = spreads.length ? spreads[Math.floor(spreads.length / 2)] : null;
    let confidence = Math.min(0.95,
      0.25 + weighted.length * 0.12 + Math.max(0, 0.32 - (spread || 0) / 40)
    );
    return { wf, nRaces: weighted.length, spread, confidence: +confidence.toFixed(2), method: 'day-consensus' };
  }


  // ═══════════════════════════════════════════════════════════════════════════
  //  blendRaceAndDayWind — combine race-level and day-level estimates
  // ═══════════════════════════════════════════════════════════════════════════
  function blendRaceAndDayWind(raceObj, dayObj) {
    if (raceObj && !dayObj) return { ...raceObj, source: 'race' };
    if (dayObj  && !raceObj) return { ...dayObj,  source: 'day' };
    if (!raceObj && !dayObj) return null;

    const agreement = Math.abs(angDiff(raceObj.wf, dayObj.wf));
    let wr = 0.70;
    const rc = raceObj.confidence || 0.3, dc = dayObj.confidence || 0.3;
    if (rc < 0.45) wr = 0.58;
    if (rc < 0.30) wr = 0.45;
    if (agreement > 12) wr = Math.max(0.45, wr - 0.10);
    if (agreement > 20) wr = Math.max(0.38, wr - 0.12);
    const wd = 1 - wr;

    return {
      wf:         circularBlend(raceObj.wf, wr, dayObj.wf, wd),
      confidence: +Math.min(0.96, rc * wr + dc * wd).toFixed(2),
      agreement:  +agreement.toFixed(1),
      raceWf: raceObj.wf, dayWf: dayObj.wf,
      source: 'blend',
      race: raceObj, day: dayObj,
      stableN: raceObj.stableN || 0
    };
  }


  // ─── EXPORTS ───────────────────────────────────────────────────────────────
  window.inferWind          = inferWind;
  window.computeDayWind     = computeDayWind;
  window.blendRaceAndDayWind = blendRaceAndDayWind;

})();
