/*
 * Page logic for the path-planning demo.
 *
 * The planners are the C++ code in planning/, compiled to WebAssembly and
 * run in planner-worker.js. This file draws the maps, takes input, and
 * replays the log each planner returns.
 *
 * Coordinates follow the C++ Node: x is the row and y is the column.
 */
(function () {
  "use strict";

  // Default points: config/main.yaml for AR0205SR. For the other maps, two
  // far-apart free cells in the map's largest connected region.
  const MAPS = [
    { file: "AR0205SR.map", dir: "bg2", start: { x: 215, y: 25 }, goal: { x: 330, y: 475 } },
    { file: "AR0072SR.map", dir: "bg2", start: { x: 60, y: 196 }, goal: { x: 102, y: 407 } },
    { file: "brc100d.map", dir: "dao", start: { x: 183, y: 224 }, goal: { x: 201, y: 53 } },
    { file: "lak505d.map", dir: "dao", start: { x: 149, y: 170 }, goal: { x: 187, y: 124 } },
    { file: "dragonfire.map", dir: "wow", start: { x: 66, y: 84 }, goal: { x: 449, y: 413 } },
    { file: "plainsofsnow.map", dir: "wow", start: { x: 81, y: 417 }, goal: { x: 438, y: 70 } },
  ];
  const FIGURE_MAP = "lak505d.map";

  const PLANNERS = {
    astar: { label: "A*", family: "grid" },
    bfs: { label: "BFS", family: "grid" },
    dfs: { label: "DFS", family: "grid" },
    rrt: { label: "RRT", family: "tree" },
    rrt_star: { label: "RRT*", family: "tree" },
  };
  const ORDER = ["astar", "bfs", "dfs", "rrt", "rrt_star"];

  // config/grid_base.yaml and config/tree_base.yaml
  const DEFAULT_PARAMS = {
    grid: { heuristic_weight: 0.7, search_space: 8 },
    tree: {
      max_iteration: 10000,
      max_branch_length: 10,
      min_branch_length: 5,
      neighbor_radius: 15,
      goal_radius: 5,
    },
  };
  const TREE_KEYS = Object.keys(DEFAULT_PARAMS.tree);

  // Logged nodes revealed per animation frame, from slowest to fastest.
  const SPEED = {
    grid: [25, 100, 400, 1500, 6000],
    tree: [3, 10, 30, 90, 300],
  };

  const FREE = 0; // planning::NodeState::kFree

  function rgba(r, g, b) {
    return ((255 << 24) | (b << 16) | (g << 8) | r) >>> 0; // ImageData, little-endian
  }

  const COLOR = {
    free: rgba(29, 82, 143),
    occupied: rgba(6, 21, 41),
    visited: rgba(74, 163, 232),
    frontier: rgba(200, 241, 255),
    orderEarly: [236, 250, 255],
    orderLate: [43, 108, 196],
    gridLine: "rgba(160, 205, 255, 0.10)",
    tree: "rgba(155, 231, 255, 0.9)",
    treeNew: "#ffffff",
    pending: "rgba(155, 231, 255, 0.65)",
    path: "#ff7a3d",
    casing: "rgba(4, 16, 36, 0.92)",
    start: "#59f0a6",
    goal: "#ffcc4d",
    goalRing: "rgba(255, 204, 77, 0.9)",
    markerEdge: "#041024",
    halo: "rgba(238, 245, 255, 0.95)",
  };

  const reduceMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;

  // ---------------------------------------------------------------------------
  // Elements

  const $ = (id) => document.getElementById(id);
  const canvas = $("map-canvas");
  const ctx = canvas.getContext("2d");
  const els = {
    app: document.querySelector(".app"),
    canvasBox: $("canvas-box"),
    overlay: $("overlay"),
    overlayText: $("overlay-text"),
    hudMap: $("hud-map"),
    hudSize: $("hud-size"),
    cursor: $("cursor"),
    hint: $("hint"),
    plannerSeg: $("planner-seg"),
    mapButton: $("map-button"),
    mapMenu: $("map-menu"),
    mapGrid: $("map-grid"),
    mapThumb: $("map-thumb"),
    mapName: $("map-name"),
    settingsButton: $("settings-button"),
    settingsMenu: $("settings-menu"),
    settingsSummary: $("settings-summary"),
    startX: $("start-x"),
    startY: $("start-y"),
    goalX: $("goal-x"),
    goalY: $("goal-y"),
    pointsNote: $("points-note"),
    paramsTitle: $("params-title"),
    paramsGrid: $("params-grid"),
    paramsTree: $("params-tree"),
    paramWeight: $("param-weight"),
    paramRadius: $("param-radius"),
    weight: $("heuristic_weight"),
    weightOut: $("weight-out"),
    resetParams: $("reset-params"),
    startLabel: $("start-label"),
    goalLabel: $("goal-label"),
    run: $("run"),
    runLabel: $("run-label"),
    pause: $("pause"),
    skip: $("skip"),
    progressBar: $("progress-bar"),
    progressText: $("progress-text"),
    speed: $("speed"),
    status: $("status"),
    statusText: $("status-text"),
    resultPlanner: $("result-planner"),
    resultLine: $("result-line"),
    statLoggedLabel: $("stat-logged-label"),
    statLogged: $("stat-logged"),
    statTime: $("stat-time"),
    statLength: $("stat-length"),
    statWaypoints: $("stat-waypoints"),
    stdout: $("stdout"),
    costChart: $("cost-chart"),
    costSvg: $("cost-svg"),
    costRange: $("cost-range"),
    compare: $("compare"),
    compareBody: $("compare-body"),
    yaml: $("yaml"),
    copyYaml: $("copy-yaml"),
  };

  // ---------------------------------------------------------------------------
  // State

  const state = {
    ready: false,
    planner: "astar",
    mapFile: MAPS[0].file,
    map: null, // { height, width, cells, base }
    start: { ...MAPS[0].start },
    goal: { ...MAPS[0].goal },
    armed: "start", // which point the next click on the map places
    params: structuredClone(DEFAULT_PARAMS),
    busy: false,
    token: 0, // bumped whenever the map or points change
    replay: null,
    compare: {},
  };

  const view = {
    k: 1, // canvas pixels per cell
    s: 1, // canvas pixels per CSS pixel
    overlay: null, // W x H visited cells for grid replays
    overlayCtx: null,
    overlayImg: null,
    overlayPx: null,
    treeLayer: document.createElement("canvas"),
  };

  // ---------------------------------------------------------------------------
  // Worker client

  function createClient() {
    const worker = new Worker("planner-worker.js");
    const pending = new Map();
    let next = 1;
    let settleReady;
    const ready = new Promise((resolve, reject) => {
      settleReady = { resolve, reject };
    });
    worker.onmessage = (event) => {
      const msg = event.data;
      if (msg.id === 0) {
        if (msg.ready) settleReady.resolve();
        else settleReady.reject(new Error(msg.error));
        return;
      }
      const call = pending.get(msg.id);
      if (!call) return;
      pending.delete(msg.id);
      if (msg.error) call.reject(new Error(msg.error));
      else call.resolve(msg.result);
    };
    worker.onerror = (event) => {
      settleReady.reject(new Error(event.message || "The planner worker failed to start."));
    };
    return {
      ready,
      call(msg) {
        return ready.then(
          () =>
            new Promise((resolve, reject) => {
              const id = next++;
              pending.set(id, { resolve, reject });
              worker.postMessage({ ...msg, id });
            })
        );
      },
    };
  }

  let client = null;
  const mapCache = new Map();

  function loadMap(file) {
    if (!mapCache.has(file)) {
      const loading = client.call({ type: "loadMap", file }).then((map) => {
        map.base = baseCanvas(map);
        return map;
      });
      loading.catch(() => mapCache.delete(file));
      mapCache.set(file, loading);
    }
    return mapCache.get(file);
  }

  function baseCanvas(map) {
    const c = document.createElement("canvas");
    c.width = map.width;
    c.height = map.height;
    const cctx = c.getContext("2d");
    const img = cctx.createImageData(map.width, map.height);
    const px = new Uint32Array(img.data.buffer);
    for (let i = 0; i < map.cells.length; i++) {
      px[i] = map.cells[i] === FREE ? COLOR.free : COLOR.occupied;
    }
    cctx.putImageData(img, 0, 0);
    return c;
  }

  function plan(planner) {
    const params = paramsFor(planner);
    const request = {
      type: "plan",
      file: state.mapFile,
      planner,
      start: { ...state.start },
      goal: { ...state.goal },
      params,
    };
    return client.call(request).then((result) =>
      Object.assign(result, { planner, mapFile: request.file, params })
    );
  }

  function paramsFor(planner) {
    return { ...state.params[PLANNERS[planner].family] };
  }

  // ---------------------------------------------------------------------------
  // Map helpers

  function mapDef(file) {
    return MAPS.find((m) => m.file === file);
  }

  function isFree(map, x, y) {
    return x >= 0 && y >= 0 && x < map.height && y < map.width && map.cells[x * map.width + y] === FREE;
  }

  // Nearest free cell, searching square rings outward.
  function nearestFree(map, x, y, maxRing = 30) {
    x = Math.min(map.height - 1, Math.max(0, x));
    y = Math.min(map.width - 1, Math.max(0, y));
    if (isFree(map, x, y)) return { x, y };
    for (let r = 1; r <= maxRing; r++) {
      let best = null;
      let bestD = Infinity;
      for (let dx = -r; dx <= r; dx++) {
        for (let dy = -r; dy <= r; dy++) {
          if (Math.max(Math.abs(dx), Math.abs(dy)) !== r) continue;
          const d = dx * dx + dy * dy;
          if (d < bestD && isFree(map, x + dx, y + dy)) {
            bestD = d;
            best = { x: x + dx, y: y + dy };
          }
        }
      }
      if (best) return best;
    }
    return null;
  }

  function pathLength(path) {
    let length = 0;
    for (let i = 2; i < path.length; i += 2) {
      length += Math.hypot(path[i] - path[i - 2], path[i + 1] - path[i - 1]);
    }
    return length;
  }

  const fmtInt = (n) => n.toLocaleString("en-US");
  const fmtMs = (ms) => (ms < 10 ? ms.toFixed(2) : ms < 100 ? ms.toFixed(1) : Math.round(ms).toString());

  // ---------------------------------------------------------------------------
  // Drawing

  function setupLayers(map) {
    view.overlay = document.createElement("canvas");
    view.overlay.width = map.width;
    view.overlay.height = map.height;
    view.overlayCtx = view.overlay.getContext("2d");
    view.overlayImg = view.overlayCtx.createImageData(map.width, map.height);
    view.overlayPx = new Uint32Array(view.overlayImg.data.buffer);
  }

  function clearOverlay() {
    if (!view.overlayPx) return;
    view.overlayPx.fill(0);
    view.overlayCtx.putImageData(view.overlayImg, 0, 0);
  }

  // On wide screens the app fills the viewport below the intro, so the
  // toolbar, the map and the replay controls are all in view at once.
  function fitApp() {
    if (window.matchMedia("(max-width: 920px)").matches) {
      els.app.style.height = "";
      return;
    }
    const top = els.app.getBoundingClientRect().top + window.scrollY;
    const available = window.innerHeight - top - 16;
    els.app.style.height = `${Math.round(Math.min(940, Math.max(520, available)))}px`;
  }

  // Fit the map inside the canvas box, keeping its aspect ratio.
  function sizeCanvas() {
    const map = state.map;
    if (!map) return;
    const boxW = els.canvasBox.clientWidth;
    const boxH = els.canvasBox.clientHeight;
    if (!boxW || !boxH) return;
    const ratio = map.width / map.height;
    const cssW = Math.max(120, Math.floor(Math.min(boxW, boxH * ratio)));
    const cssH = Math.floor(cssW / ratio);
    canvas.style.width = `${cssW}px`;
    canvas.style.height = `${cssH}px`;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const k = Math.max(1, Math.min(8, Math.ceil((cssW * dpr) / map.width)));
    if (canvas.width !== map.width * k || canvas.height !== map.height * k) {
      canvas.width = map.width * k;
      canvas.height = map.height * k;
      view.k = k;
      rebuildTreeLayer();
    }
    view.s = canvas.width / cssW;
    draw();
  }

  function cellCenter(x, y) {
    return [(y + 0.5) * view.k, (x + 0.5) * view.k];
  }

  function draw() {
    const map = state.map;
    if (!map) return;
    const r = state.replay;
    ctx.imageSmoothingEnabled = false;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.drawImage(map.base, 0, 0, canvas.width, canvas.height);

    if (r && r.kind === "grid") {
      ctx.drawImage(view.overlay, 0, 0, canvas.width, canvas.height);
    } else if (r && r.kind === "tree") {
      ctx.drawImage(view.treeLayer, 0, 0);
      drawTreeExtras(r);
    }

    drawGridLines(map);

    const family = r ? r.kind : PLANNERS[state.planner].family;
    if (family === "tree") drawGoalRadius(r ? r.result.params.goal_radius : state.params.tree.goal_radius);
    if (r && r.pathShown > 1) drawPath(r.result.path, r.pathShown);

    drawMarker(state.start, COLOR.start, state.armed === "start");
    drawMarker(state.goal, COLOR.goal, state.armed === "goal");
  }

  // Faint reference lines every 64 cells, like a drawing grid.
  function drawGridLines(map) {
    ctx.save();
    ctx.strokeStyle = COLOR.gridLine;
    ctx.lineWidth = Math.max(1, view.s * 0.8);
    ctx.beginPath();
    for (let y = 64; y < map.width; y += 64) {
      ctx.moveTo(y * view.k, 0);
      ctx.lineTo(y * view.k, canvas.height);
    }
    for (let x = 64; x < map.height; x += 64) {
      ctx.moveTo(0, x * view.k);
      ctx.lineTo(canvas.width, x * view.k);
    }
    ctx.stroke();
    ctx.restore();
  }

  function drawPath(path, points) {
    ctx.save();
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.beginPath();
    for (let i = 0; i < points; i++) {
      const [cx, cy] = cellCenter(path[2 * i], path[2 * i + 1]);
      if (i === 0) ctx.moveTo(cx, cy);
      else ctx.lineTo(cx, cy);
    }
    const width = Math.max(2.4 * view.s, 0.7 * view.k);
    ctx.strokeStyle = COLOR.casing;
    ctx.lineWidth = width + 3 * view.s;
    ctx.stroke();
    ctx.strokeStyle = COLOR.path;
    ctx.lineWidth = width;
    ctx.stroke();
    ctx.restore();
  }

  function drawGoalRadius(radius) {
    const [cx, cy] = cellCenter(state.goal.x, state.goal.y);
    ctx.save();
    ctx.beginPath();
    ctx.arc(cx, cy, radius * view.k, 0, Math.PI * 2);
    ctx.setLineDash([3 * view.s, 3 * view.s]);
    ctx.lineWidth = 1.3 * view.s;
    ctx.strokeStyle = COLOR.goalRing;
    ctx.stroke();
    ctx.restore();
  }

  function drawMarker(point, color, armed) {
    const [cx, cy] = cellCenter(point.x, point.y);
    const r = 6.5 * view.s;
    ctx.save();
    if (armed) {
      ctx.beginPath();
      ctx.arc(cx, cy, r + 6 * view.s, 0, Math.PI * 2);
      ctx.setLineDash([2.5 * view.s, 2.5 * view.s]);
      ctx.lineWidth = 1.4 * view.s;
      ctx.strokeStyle = color;
      ctx.stroke();
      ctx.setLineDash([]);
    }
    ctx.beginPath();
    ctx.arc(cx, cy, r + 1.8 * view.s, 0, Math.PI * 2);
    ctx.fillStyle = COLOR.halo;
    ctx.fill();
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
    ctx.lineWidth = 2 * view.s;
    ctx.strokeStyle = COLOR.markerEdge;
    ctx.stroke();
    ctx.restore();
  }

  // ---------------------------------------------------------------------------
  // Replay of a planner log

  function startReplay(result) {
    stopReplay();
    const kind = PLANNERS[result.planner].family;
    const r = {
      kind,
      result,
      total: result.log.length / 3,
      i: 0,
      faded: 0,
      pathTotal: result.path.length / 2,
      pathShown: 0,
      done: false,
      paused: false,
      raf: 0,
    };
    state.replay = r;
    if (kind === "grid") clearOverlay();
    else prepareTree(r);
    els.pause.disabled = false;
    els.skip.disabled = false;
    setPauseIcon(false);
    els.progressBar.classList.remove("done");
    setStatus("replaying", "replaying the log");
    updateProgress(r);
    if (reduceMotion) {
      skipReplay();
    } else {
      r.raf = requestAnimationFrame(tick);
    }
  }

  function stopReplay() {
    const r = state.replay;
    if (r && r.raf) cancelAnimationFrame(r.raf);
    state.replay = null;
    els.pause.disabled = true;
    els.skip.disabled = true;
    setPauseIcon(false);
    clearOverlay();
    clearTreeLayer();
    els.progressBar.style.width = "0";
    els.progressBar.classList.remove("done");
    els.progressText.textContent = "replay idle";
  }

  function speedLevel() {
    return Number(els.speed.value) - 1;
  }

  function tick() {
    const r = state.replay;
    if (!r || r.paused || r.done) return;
    const level = speedLevel();
    if (r.i < r.total) {
      advance(r, SPEED[r.kind][level]);
    } else if (r.pathShown < r.pathTotal) {
      const step = level === 4 ? r.pathTotal : Math.max(1, Math.ceil(r.pathTotal / 36));
      r.pathShown = Math.min(r.pathTotal, r.pathShown + step);
    }
    if (r.i >= r.total && r.pathShown >= r.pathTotal) {
      finishReplay(r);
    } else {
      r.raf = requestAnimationFrame(tick);
    }
    updateProgress(r);
    draw();
  }

  function advance(r, rate) {
    if (r.kind === "grid") advanceGrid(r, rate);
    else advanceTree(r, rate);
  }

  function skipReplay() {
    const r = state.replay;
    if (!r) return;
    if (r.raf) cancelAnimationFrame(r.raf);
    advance(r, Infinity);
    r.pathShown = r.pathTotal;
    finishReplay(r);
    updateProgress(r);
    draw();
  }

  function finishReplay(r) {
    r.done = true;
    r.raf = 0;
    els.pause.disabled = true;
    els.skip.disabled = true;
    els.progressBar.classList.add("done");
    if (r.result.found) setStatus("found", "path found");
    else setStatus("nopath", "no path");
  }

  function setPauseIcon(paused) {
    els.pause.innerHTML = `<svg class="icon${paused ? " icon-fill" : ""}" aria-hidden="true"><use href="#${paused ? "i-play" : "i-pause"}"/></svg>`;
    els.pause.setAttribute("aria-label", paused ? "Resume replay" : "Pause replay");
  }

  function togglePause() {
    const r = state.replay;
    if (!r || r.done) return;
    r.paused = !r.paused;
    setPauseIcon(r.paused);
    if (r.paused) {
      setStatus("paused", "paused");
    } else {
      setStatus("replaying", "replaying the log");
      r.raf = requestAnimationFrame(tick);
    }
  }

  function updateProgress(r) {
    const total = r.total + r.pathTotal;
    const done = r.i + r.pathShown;
    els.progressBar.style.width = `${total ? (100 * done) / total : 100}%`;
    const noun = r.kind === "grid" ? "expanded" : "tree nodes";
    els.progressText.textContent = `${fmtInt(r.i)} / ${fmtInt(r.total)} ${noun}`;
  }

  // Grid planners log each cell when they expand it.
  function advanceGrid(r, rate) {
    const log = r.result.log;
    const px = view.overlayPx;
    const W = state.map.width;
    const end = Math.min(r.total, r.i + rate);
    for (let j = r.i; j < end; j++) {
      px[log[3 * j] * W + log[3 * j + 1]] = COLOR.frontier;
    }
    r.i = end;
    // Cells expanded in the last few frames stay bright: the search front.
    const fadeTo = r.i >= r.total ? r.total : Math.max(0, r.i - Math.min(rate, 4000) * 6);
    for (let j = r.faded; j < fadeTo; j++) {
      px[log[3 * j] * W + log[3 * j + 1]] = COLOR.visited;
    }
    r.faded = Math.max(r.faded, fadeTo);
    view.overlayCtx.putImageData(view.overlayImg, 0, 0);
  }

  // Tree planners log each node when it joins the tree. An edge is drawn
  // once both of its ends are in view. RRT* rewires, and its log holds the
  // final parents, so some parents appear after their children.
  function prepareTree(r) {
    const log = r.result.log;
    const edges = [];
    for (let j = 0; j < r.total; j++) {
      if (log[3 * j + 2] >= 0) edges.push(j);
    }
    const revealAt = (j) => Math.max(j, log[3 * j + 2]);
    edges.sort((a, b) => revealAt(a) - revealAt(b));
    r.edges = Int32Array.from(edges);
    r.edgeAt = Int32Array.from(edges, revealAt);
    r.drawn = 0;
    rebuildTreeLayer();
  }

  function clearTreeLayer() {
    const t = view.treeLayer;
    t.getContext("2d").clearRect(0, 0, t.width, t.height);
  }

  function rebuildTreeLayer() {
    const t = view.treeLayer;
    if (t.width !== canvas.width || t.height !== canvas.height) {
      t.width = canvas.width;
      t.height = canvas.height;
    } else {
      clearTreeLayer();
    }
    const r = state.replay;
    if (r && r.kind === "tree") drawEdges(r, 0, r.drawn);
  }

  function drawEdges(r, from, to) {
    if (to <= from) return;
    const tctx = view.treeLayer.getContext("2d");
    const log = r.result.log;
    tctx.save();
    tctx.strokeStyle = COLOR.tree;
    tctx.lineWidth = Math.max(1, 0.95 * view.s);
    tctx.lineCap = "round";
    tctx.beginPath();
    for (let e = from; e < to; e++) {
      const j = r.edges[e];
      const p = log[3 * j + 2];
      const [ax, ay] = cellCenter(log[3 * p], log[3 * p + 1]);
      const [bx, by] = cellCenter(log[3 * j], log[3 * j + 1]);
      tctx.moveTo(ax, ay);
      tctx.lineTo(bx, by);
    }
    tctx.stroke();
    tctx.restore();
  }

  function advanceTree(r, rate) {
    r.i = Math.min(r.total, r.i + rate);
    const before = r.drawn;
    while (r.drawn < r.edges.length && r.edgeAt[r.drawn] < r.i) r.drawn++;
    drawEdges(r, before, r.drawn);
  }

  function drawTreeExtras(r) {
    const log = r.result.log;
    const dot = Math.max(1.4 * view.s, 0.45 * view.k);
    // Nodes whose final parent has not appeared yet (rewired by RRT*).
    ctx.save();
    ctx.fillStyle = COLOR.pending;
    ctx.beginPath();
    for (let j = 1; j < r.i; j++) {
      const p = log[3 * j + 2];
      if (p >= r.i || p === -2) {
        const [cx, cy] = cellCenter(log[3 * j], log[3 * j + 1]);
        ctx.moveTo(cx + dot, cy);
        ctx.arc(cx, cy, dot, 0, Math.PI * 2);
      }
    }
    ctx.fill();
    if (!r.done) {
      ctx.fillStyle = COLOR.treeNew;
      ctx.beginPath();
      for (let j = Math.max(0, r.i - 3); j < r.i; j++) {
        const [cx, cy] = cellCenter(log[3 * j], log[3 * j + 1]);
        ctx.moveTo(cx + dot * 1.8, cy);
        ctx.arc(cx, cy, dot * 1.8, 0, Math.PI * 2);
      }
      ctx.fill();
    }
    ctx.restore();
  }

  // ---------------------------------------------------------------------------
  // Status and results

  function setStatus(kind, text) {
    els.status.dataset.state = kind;
    els.statusText.textContent = text;
  }

  function setBusy(busy) {
    state.busy = busy;
    const off = busy || !state.ready || !state.map;
    els.run.disabled = off;
    els.compare.disabled = off;
    els.runLabel.textContent = busy ? "Running…" : `Run ${PLANNERS[state.planner].label}`;
  }

  function resetResult() {
    els.resultPlanner.textContent = "no run yet";
    els.resultLine.hidden = true;
    els.statLoggedLabel.textContent = "Logged nodes";
    for (const el of [els.statLogged, els.statTime, els.statLength, els.statWaypoints]) el.textContent = "—";
    els.stdout.innerHTML = '<span class="quiet">Run a planner to see what the C++ prints.</span>';
    els.costChart.hidden = true;
  }

  function showResult(result) {
    const p = PLANNERS[result.planner];
    const total = result.log.length / 3;
    els.resultPlanner.textContent = `${p.label} · ${result.mapFile.replace(".map", "")}`;
    els.resultLine.hidden = false;
    els.resultLine.textContent = result.found
      ? `Path found: ${pathLength(result.path).toFixed(1)} cells long.`
      : "No path between these points.";
    els.resultLine.classList.toggle("none", !result.found);
    els.statLoggedLabel.textContent = p.family === "grid" ? "Expanded cells" : "Tree nodes";
    els.statLogged.textContent = fmtInt(total);
    els.statTime.innerHTML = `${fmtMs(result.timeMs)}<small> ms</small>`;
    els.statLength.innerHTML = result.found ? `${pathLength(result.path).toFixed(1)}<small> cells</small>` : "—";
    els.statWaypoints.textContent = result.found ? fmtInt(result.path.length / 2) : "—";

    const lines = result.stdout || [];
    if (lines.length) els.stdout.textContent = lines.join("\n");
    else els.stdout.innerHTML = '<span class="quiet">(nothing printed)</span>';
    drawCostChart(result);
  }

  // RRTStar::FindPath prints "Goal cost: <f>\tTime: <ms> ms" whenever
  // rewiring improves the path.
  function drawCostChart(result) {
    const points = [];
    for (const line of result.stdout || []) {
      const m = /Goal cost: ([\d.]+)\s+Time: (\d+) ms/.exec(line);
      if (m) points.push([Number(m[2]), Number(m[1])]);
    }
    if (points.length < 2) {
      els.costChart.hidden = true;
      return;
    }
    const W = 300;
    const H = 64;
    const pad = 6;
    const tMax = Math.max(points[points.length - 1][0], result.timeMs, 1);
    const costs = points.map((p) => p[1]);
    const cMin = Math.min(...costs);
    const cMax = Math.max(...costs);
    const sx = (t) => pad + (t / tMax) * (W - 2 * pad);
    const sy = (c) => (cMax === cMin ? H / 2 : pad + ((cMax - c) / (cMax - cMin)) * (H - 2 * pad));
    let d = `M${sx(points[0][0]).toFixed(1)} ${sy(points[0][1]).toFixed(1)}`;
    for (let i = 1; i < points.length; i++) {
      d += ` H${sx(points[i][0]).toFixed(1)} V${sy(points[i][1]).toFixed(1)}`;
    }
    d += ` H${sx(tMax).toFixed(1)}`;
    const area = `${d} V${H} H${sx(points[0][0]).toFixed(1)} Z`;
    els.costSvg.innerHTML =
      `<path d="${area}" fill="rgba(255, 122, 61, 0.14)"/>` +
      `<path d="${d}" fill="none" stroke="#ff7a3d" stroke-width="2" vector-effect="non-scaling-stroke"/>`;
    els.costRange.textContent = `${cMax.toFixed(0)} → ${cMin.toFixed(0)} in ${Math.round(tMax)} ms`;
    els.costChart.hidden = false;
  }

  // ---------------------------------------------------------------------------
  // Run and compare

  async function runPlanner() {
    if (state.busy || !state.ready || !state.map) return;
    closeMenus();
    const planner = state.planner;
    const token = state.token;
    stopReplay();
    draw();
    setBusy(true);
    setStatus("planning", `running ${PLANNERS[planner].label} in C++`);
    try {
      const result = await plan(planner);
      if (token !== state.token) return;
      showResult(result);
      startReplay(result);
    } catch (err) {
      setStatus("error", err.message);
    } finally {
      setBusy(false);
    }
  }

  async function compareAll() {
    if (state.busy || !state.ready || !state.map) return;
    closeMenus();
    const token = state.token;
    stopReplay();
    draw();
    setBusy(true);
    state.compare = {};
    els.compareBody.innerHTML = ORDER.map(
      (planner) =>
        `<tr data-planner="${planner}"><td>${PLANNERS[planner].label}</td><td colspan="3" class="wait">running…</td></tr>`
    ).join("");
    try {
      for (const planner of ORDER) {
        setStatus("planning", `comparing · ${PLANNERS[planner].label}`);
        const result = await plan(planner);
        if (token !== state.token) return;
        state.compare[planner] = result;
        renderCompareRow(planner);
      }
      markBest();
      setStatus("ready", "compared · click a row");
    } catch (err) {
      setStatus("error", err.message);
    } finally {
      setBusy(false);
    }
  }

  function renderCompareRow(planner) {
    const r = state.compare[planner];
    const row = els.compareBody.querySelector(`tr[data-planner="${planner}"]`);
    if (!row || !r) return;
    row.innerHTML =
      `<td><button class="row-btn" type="button" data-replay="${planner}" title="Replay this run">${PLANNERS[planner].label}</button></td>` +
      (r.found
        ? `<td data-col="length">${pathLength(r.path).toFixed(0)}</td>`
        : `<td data-col="length" class="none">none</td>`) +
      `<td data-col="logged">${fmtInt(r.log.length / 3)}</td>` +
      `<td data-col="time">${fmtMs(r.timeMs)}</td>`;
  }

  function markBest() {
    const entries = ORDER.map((p) => [p, state.compare[p]]).filter(([, r]) => r);
    const best = (col, value, filter = () => true) => {
      const candidates = entries.filter(([, r]) => filter(r));
      if (!candidates.length) return;
      const [planner] = candidates.reduce((a, b) => (value(b[1]) < value(a[1]) ? b : a));
      const cell = els.compareBody.querySelector(`tr[data-planner="${planner}"] td[data-col="${col}"]`);
      if (cell) {
        cell.classList.add("best");
        cell.title = { length: "shortest path", logged: "fewest logged nodes", time: "fastest" }[col];
      }
    };
    best("length", (r) => pathLength(r.path), (r) => r.found);
    best("logged", (r) => r.log.length);
    best("time", (r) => r.timeMs);
  }

  function replayFromCompare(planner) {
    const result = state.compare[planner];
    if (!result || state.busy) return;
    selectPlanner(planner);
    showResult(result);
    startReplay(result);
  }

  function clearCompare() {
    state.compare = {};
    els.compareBody.innerHTML =
      '<tr class="empty"><td colspan="4">Runs every planner on these points. Click a row to replay it.</td></tr>';
  }

  // ---------------------------------------------------------------------------
  // Points, planner, map and parameters

  // Anything that changes the problem makes the shown result stale.
  function invalidate() {
    state.token++;
    if (state.replay) stopReplay();
    resetResult();
    clearCompare();
    if (state.ready && !state.busy) setStatus("ready", "ready");
  }

  function setPoint(which, point) {
    if (state[which].x === point.x && state[which].y === point.y) return;
    state[which] = { x: point.x, y: point.y };
    invalidate();
    syncPoints();
    draw();
  }

  function arm(which) {
    state.armed = which;
    document.querySelectorAll("[data-point]").forEach((chip) => {
      chip.setAttribute("aria-pressed", String(chip.dataset.point === which));
    });
    els.hint.textContent = `Click the map to place the ${which}. Drag a marker to move it.`;
    draw();
  }

  function syncPoints() {
    els.startX.value = state.start.x;
    els.startY.value = state.start.y;
    els.goalX.value = state.goal.x;
    els.goalY.value = state.goal.y;
    els.startLabel.textContent = `${state.start.x}, ${state.start.y}`;
    els.goalLabel.textContent = `${state.goal.x}, ${state.goal.y}`;
    const map = state.map;
    if (map) {
      for (const el of [els.startX, els.goalX]) el.max = map.height - 1;
      for (const el of [els.startY, els.goalY]) el.max = map.width - 1;
      canvas.setAttribute(
        "aria-label",
        `Map ${state.mapFile}, ${map.height} rows by ${map.width} columns. ` +
          `Start at x ${state.start.x}, y ${state.start.y}. Goal at x ${state.goal.x}, y ${state.goal.y}.`
      );
    }
    updateYaml();
  }

  function pointFromInputs(which) {
    const map = state.map;
    if (!map) return;
    const xEl = which === "start" ? els.startX : els.goalX;
    const yEl = which === "start" ? els.startY : els.goalY;
    const x = Math.round(Number(xEl.value));
    const y = Math.round(Number(yEl.value));
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      syncPoints();
      return;
    }
    const free = nearestFree(map, x, y, 96);
    if (!free) {
      els.pointsNote.textContent = `No free cell near x ${x}, y ${y}.`;
      syncPoints();
      return;
    }
    els.pointsNote.textContent =
      free.x !== x || free.y !== y ? `Moved the ${which} to the nearest free cell.` : "";
    setPoint(which, free);
    syncPoints();
  }

  function selectPlanner(planner) {
    state.planner = planner;
    els.plannerSeg.querySelectorAll("[data-planner]").forEach((b) => {
      b.setAttribute("aria-checked", String(b.dataset.planner === planner));
    });
    const family = PLANNERS[planner].family;
    els.paramsGrid.hidden = family !== "grid";
    els.paramsTree.hidden = family !== "tree";
    els.paramWeight.hidden = planner !== "astar";
    els.paramRadius.hidden = planner !== "rrt_star";
    els.paramsTitle.textContent = family === "grid" ? "config/grid_base.yaml" : "config/tree_base.yaml";
    setBusy(state.busy);
    updateSummary();
    updateYaml();
    if (!state.replay) draw();
  }

  function updateSummary() {
    const g = state.params.grid;
    const t = state.params.tree;
    const iter = t.max_iteration >= 1000 ? `${t.max_iteration / 1000}k` : String(t.max_iteration);
    const text = {
      astar: `w ${g.heuristic_weight} · ${g.search_space}-dir`,
      bfs: `${g.search_space}-dir`,
      dfs: `${g.search_space}-dir`,
      rrt: `${iter} iter · step ${t.max_branch_length}`,
      rrt_star: `${iter} iter · r ${t.neighbor_radius}`,
    }[state.planner];
    els.settingsSummary.textContent = text;
  }

  async function selectMap(file) {
    const def = mapDef(file);
    state.mapFile = file;
    state.start = { ...def.start };
    state.goal = { ...def.goal };
    els.pointsNote.textContent = "";
    invalidate();
    els.mapName.textContent = file.replace(".map", "");
    els.hudMap.textContent = file;
    els.mapGrid.querySelectorAll("[data-map]").forEach((b) => {
      b.setAttribute("aria-checked", String(b.dataset.map === file));
    });
    setStatus("loading", "loading the map");
    try {
      const map = await loadMap(file);
      if (state.mapFile !== file) return;
      state.map = map;
      setupLayers(map);
      els.hudSize.textContent = `${map.height} × ${map.width}`;
      drawThumbInto(els.mapThumb, map);
      syncPoints();
      sizeCanvas();
      arm("start");
      setBusy(state.busy);
      if (!state.busy) setStatus("ready", "ready");
    } catch (err) {
      setStatus("error", err.message);
    }
  }

  function updateYaml() {
    const def = mapDef(state.mapFile);
    const family = PLANNERS[state.planner].family;
    const params = state.params[family];
    const lines = [
      "# config/main.yaml",
      `map: "/${def.dir}/${def.file}"`,
      `planner_name: "${state.planner}"`,
      "path:",
      "  start:",
      `    x: ${state.start.x}`,
      `    y: ${state.start.y}`,
      "  goal:",
      `    x: ${state.goal.x}`,
      `    y: ${state.goal.y}`,
      "",
      family === "grid" ? "# config/grid_base.yaml" : "# config/tree_base.yaml",
      ...Object.entries(params).map(([key, value]) => `${key}: ${value}`),
    ];
    els.yaml.dataset.text = lines.join("\n");
    els.yaml.innerHTML = lines.map(highlightYaml).join("\n");
  }

  function escapeHtml(s) {
    return s.replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);
  }

  function highlightYaml(line) {
    if (line.startsWith("#")) return `<span class="yaml-comment">${escapeHtml(line)}</span>`;
    const m = /^(\s*)([\w]+):(.*)$/.exec(line);
    if (!m) return escapeHtml(line);
    return `${m[1]}<span class="yaml-key">${m[2]}</span>:<span class="yaml-value">${escapeHtml(m[3])}</span>`;
  }

  function readParams() {
    state.params.grid.heuristic_weight = Number(els.weight.value);
    for (const key of TREE_KEYS) {
      const el = $(key);
      let value = Math.round(Number(el.value));
      if (!Number.isFinite(value)) value = DEFAULT_PARAMS.tree[key];
      value = Math.min(Number(el.max), Math.max(Number(el.min), value));
      el.value = value;
      state.params.tree[key] = value;
    }
    els.weightOut.textContent = state.params.grid.heuristic_weight.toFixed(2);
    updateSummary();
    updateYaml();
    if (!state.replay) draw();
  }

  function setSearchSpace(space) {
    state.params.grid.search_space = space;
    document.querySelectorAll("[data-space]").forEach((b) => {
      b.setAttribute("aria-checked", String(Number(b.dataset.space) === space));
    });
    updateSummary();
    updateYaml();
  }

  function writeParams(params) {
    els.weight.value = params.grid.heuristic_weight;
    for (const key of TREE_KEYS) $(key).value = params.tree[key];
    setSearchSpace(params.grid.search_space);
    readParams();
  }

  // ---------------------------------------------------------------------------
  // Menus

  const menus = [
    [els.mapButton, els.mapMenu],
    [els.settingsButton, els.settingsMenu],
  ];

  function closeMenus(except) {
    for (const [button, menu] of menus) {
      if (menu === except) continue;
      menu.hidden = true;
      button.setAttribute("aria-expanded", "false");
    }
  }

  function toggleMenu(button, menu) {
    const open = menu.hidden;
    closeMenus(menu);
    menu.hidden = !open;
    button.setAttribute("aria-expanded", String(open));
  }

  // ---------------------------------------------------------------------------
  // Canvas input

  const pointer = { drag: null, down: null };

  function cellFromEvent(event) {
    const rect = canvas.getBoundingClientRect();
    const map = state.map;
    const y = Math.floor(((event.clientX - rect.left) / rect.width) * map.width);
    const x = Math.floor(((event.clientY - rect.top) / rect.height) * map.height);
    return { x, y };
  }

  function markerUnder(event) {
    const rect = canvas.getBoundingClientRect();
    const map = state.map;
    const hit = (p) => {
      const cx = rect.left + ((p.y + 0.5) / map.width) * rect.width;
      const cy = rect.top + ((p.x + 0.5) / map.height) * rect.height;
      return Math.hypot(event.clientX - cx, event.clientY - cy) <= 12;
    };
    if (hit(state.goal)) return "goal";
    if (hit(state.start)) return "start";
    return null;
  }

  canvas.addEventListener("pointerdown", (event) => {
    if (!state.map || event.button !== 0) return;
    closeMenus();
    const marker = markerUnder(event);
    if (marker) {
      pointer.drag = marker;
      canvas.setPointerCapture(event.pointerId);
      canvas.classList.add("dragging");
      event.preventDefault();
    } else {
      pointer.down = { x: event.clientX, y: event.clientY };
    }
  });

  canvas.addEventListener("pointermove", (event) => {
    if (!state.map) return;
    const cell = cellFromEvent(event);
    const inside = cell.x >= 0 && cell.y >= 0 && cell.x < state.map.height && cell.y < state.map.width;
    els.cursor.textContent = inside
      ? `x ${cell.x} · y ${cell.y}${isFree(state.map, cell.x, cell.y) ? "" : " · wall"}`
      : "";
    if (pointer.drag) {
      const free = nearestFree(state.map, cell.x, cell.y, 12);
      if (free) setPoint(pointer.drag, free);
    } else {
      canvas.classList.toggle("over-marker", !!markerUnder(event));
    }
  });

  function endPointer(event) {
    if (pointer.drag) {
      pointer.drag = null;
      canvas.classList.remove("dragging");
      if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
      return;
    }
    const down = pointer.down;
    pointer.down = null;
    if (!down || event.type !== "pointerup") return;
    if (Math.hypot(event.clientX - down.x, event.clientY - down.y) > 6) return;
    const cell = cellFromEvent(event);
    const free = nearestFree(state.map, cell.x, cell.y);
    if (!free) return;
    const which = state.armed;
    els.pointsNote.textContent = "";
    setPoint(which, free);
    arm(which === "start" ? "goal" : "start");
  }

  canvas.addEventListener("pointerup", endPointer);
  canvas.addEventListener("pointercancel", endPointer);
  canvas.addEventListener("pointerleave", () => {
    els.cursor.textContent = "";
  });

  // ---------------------------------------------------------------------------
  // Map picker and figures

  function buildMapGrid() {
    els.mapGrid.innerHTML = MAPS.map(
      (m) =>
        `<button class="map-option" type="button" role="radio" data-map="${m.file}" aria-checked="${m.file === state.mapFile}">` +
        `<canvas width="112" height="112" aria-hidden="true"></canvas>` +
        `<strong>${m.file.replace(".map", "")}</strong><small>${m.dir} · <span data-size>…</span></small></button>`
    ).join("");
    MAPS.forEach((m) => {
      const button = els.mapGrid.querySelector(`[data-map="${m.file}"]`);
      m.thumb = button.querySelector("canvas");
      m.sizeEl = button.querySelector("[data-size]");
    });
  }

  function drawThumbInto(c, map) {
    const tctx = c.getContext("2d");
    tctx.fillStyle = "#061529";
    tctx.fillRect(0, 0, c.width, c.height);
    const scale = Math.min(c.width / map.width, c.height / map.height);
    const w = map.width * scale;
    const h = map.height * scale;
    tctx.imageSmoothingEnabled = true;
    tctx.imageSmoothingQuality = "high";
    tctx.drawImage(map.base, (c.width - w) / 2, (c.height - h) / 2, w, h);
  }

  async function loadThumbnails() {
    for (const def of MAPS) {
      try {
        const map = await loadMap(def.file);
        drawThumbInto(def.thumb, map);
        def.sizeEl.textContent = `${map.height}×${map.width}`;
      } catch {
        def.sizeEl.textContent = "unavailable";
      }
    }
  }

  // Expansion order of the grid planners, from the C++ logs, for the
  // planner cards.
  async function renderOrderFigures() {
    const canvases = document.querySelectorAll("canvas[data-order]");
    if (!canvases.length) return;
    const def = mapDef(FIGURE_MAP);
    const map = await loadMap(def.file);
    const [r0, g0, b0] = COLOR.orderEarly;
    const [r1, g1, b1] = COLOR.orderLate;
    for (const c of canvases) {
      const planner = c.dataset.order;
      const result = await client.call({
        type: "plan",
        file: def.file,
        planner,
        start: def.start,
        goal: def.goal,
        params: DEFAULT_PARAMS.grid,
      });
      const k = 3;
      c.width = map.width * k;
      c.height = map.height * k;
      c.style.aspectRatio = `${map.width} / ${map.height}`;
      const layer = document.createElement("canvas");
      layer.width = map.width;
      layer.height = map.height;
      const lctx = layer.getContext("2d");
      const img = lctx.createImageData(map.width, map.height);
      const px = new Uint32Array(img.data.buffer);
      for (let i = 0; i < map.cells.length; i++) {
        px[i] = map.cells[i] === FREE ? COLOR.free : COLOR.occupied;
      }
      const n = result.log.length / 3;
      for (let j = 0; j < n; j++) {
        const t = n > 1 ? j / (n - 1) : 0;
        px[result.log[3 * j] * map.width + result.log[3 * j + 1]] = rgba(
          Math.round(r0 + (r1 - r0) * t),
          Math.round(g0 + (g1 - g0) * t),
          Math.round(b0 + (b1 - b0) * t)
        );
      }
      lctx.putImageData(img, 0, 0);
      const fctx = c.getContext("2d");
      fctx.imageSmoothingEnabled = false;
      fctx.drawImage(layer, 0, 0, c.width, c.height);
      fctx.lineJoin = "round";
      fctx.lineCap = "round";
      fctx.beginPath();
      for (let i = 0; i < result.path.length; i += 2) {
        const cx = (result.path[i + 1] + 0.5) * k;
        const cy = (result.path[i] + 0.5) * k;
        if (i === 0) fctx.moveTo(cx, cy);
        else fctx.lineTo(cx, cy);
      }
      fctx.strokeStyle = COLOR.casing;
      fctx.lineWidth = 5.5;
      fctx.stroke();
      fctx.strokeStyle = COLOR.path;
      fctx.lineWidth = 2.6;
      fctx.stroke();
      for (const [p, color] of [[def.start, COLOR.start], [def.goal, COLOR.goal]]) {
        fctx.beginPath();
        fctx.arc((p.y + 0.5) * k, (p.x + 0.5) * k, 7, 0, Math.PI * 2);
        fctx.fillStyle = color;
        fctx.fill();
        fctx.lineWidth = 2;
        fctx.strokeStyle = COLOR.markerEdge;
        fctx.stroke();
      }
      const caption = document.querySelector(`figcaption[data-caption="${planner}"]`);
      if (caption) {
        caption.textContent =
          `lak505d, from the C++ log: ${fmtInt(n)} cells expanded, bright first, deep blue last. ` +
          `Path: ${fmtInt(result.path.length / 2 - 1)} moves.`;
      }
    }
  }

  // ---------------------------------------------------------------------------
  // Wiring

  function tryPlanner(planner) {
    selectPlanner(planner);
    document.getElementById("demo").scrollIntoView({ behavior: reduceMotion ? "auto" : "smooth", block: "start" });
    runPlanner();
  }

  function isTyping(target) {
    return target && (target.tagName === "INPUT" || target.tagName === "TEXTAREA" || target.isContentEditable);
  }

  function wire() {
    els.plannerSeg.addEventListener("click", (event) => {
      const button = event.target.closest("[data-planner]");
      if (button) selectPlanner(button.dataset.planner);
    });
    els.mapButton.addEventListener("click", () => toggleMenu(els.mapButton, els.mapMenu));
    els.settingsButton.addEventListener("click", () => toggleMenu(els.settingsButton, els.settingsMenu));
    els.mapGrid.addEventListener("click", (event) => {
      const button = event.target.closest("[data-map]");
      if (!button) return;
      closeMenus();
      if (button.dataset.map !== state.mapFile) selectMap(button.dataset.map);
    });
    document.addEventListener("click", (event) => {
      if (!event.target.closest(".dropdown")) closeMenus();
    });
    for (const which of ["start", "goal"]) {
      for (const axis of ["x", "y"]) {
        $(`${which}-${axis}`).addEventListener("change", () => pointFromInputs(which));
      }
    }
    document.querySelectorAll("[data-point]").forEach((chip) => {
      chip.addEventListener("click", () => arm(chip.dataset.point));
    });
    $("swap").addEventListener("click", () => {
      const { start, goal } = state;
      state.start = goal;
      state.goal = start;
      invalidate();
      syncPoints();
      draw();
    });
    els.weight.addEventListener("input", readParams);
    document.querySelectorAll("[data-space]").forEach((b) => {
      b.addEventListener("click", () => setSearchSpace(Number(b.dataset.space)));
    });
    for (const key of TREE_KEYS) $(key).addEventListener("change", readParams);
    els.resetParams.addEventListener("click", () => writeParams(structuredClone(DEFAULT_PARAMS)));
    els.run.addEventListener("click", runPlanner);
    els.pause.addEventListener("click", togglePause);
    els.skip.addEventListener("click", skipReplay);
    els.compare.addEventListener("click", compareAll);
    els.compareBody.addEventListener("click", (event) => {
      const button = event.target.closest("[data-replay]");
      if (button) replayFromCompare(button.dataset.replay);
    });
    els.copyYaml.addEventListener("click", async () => {
      try {
        await navigator.clipboard.writeText(els.yaml.dataset.text);
        els.copyYaml.textContent = "Copied";
      } catch {
        els.copyYaml.textContent = "Select and copy";
      }
      setTimeout(() => (els.copyYaml.textContent = "Copy"), 1600);
    });
    document.querySelectorAll("[data-try]").forEach((button) => {
      button.addEventListener("click", () => tryPlanner(button.dataset.try));
    });
    document.addEventListener("keydown", (event) => {
      if (event.key === "Escape") {
        closeMenus();
        return;
      }
      if (event.metaKey || event.ctrlKey || event.altKey || isTyping(event.target)) return;
      if (event.key === "r" || event.key === "R") {
        event.preventDefault();
        runPlanner();
      } else if (event.key === " " && state.replay && !state.replay.done) {
        event.preventDefault();
        togglePause();
      } else if (/^[1-5]$/.test(event.key)) {
        selectPlanner(ORDER[Number(event.key) - 1]);
      }
    });
    new ResizeObserver(sizeCanvas).observe(els.canvasBox);
    window.addEventListener("resize", fitApp);
  }

  async function init() {
    fitApp();
    buildMapGrid();
    wire();
    writeParams(state.params);
    selectPlanner(state.planner);
    syncPoints();
    arm("start");
    resetResult();

    if (typeof Worker === "undefined" || typeof WebAssembly === "undefined") {
      els.overlayText.textContent = "This browser cannot run WebAssembly in a worker, so the demo is unavailable.";
      setStatus("error", "unsupported browser");
      return;
    }

    client = createClient();
    try {
      await client.ready;
      state.ready = true;
      await selectMap(state.mapFile);
      els.overlay.hidden = true;
      setBusy(false);
    } catch (err) {
      els.overlayText.textContent = `Could not load the C++ planners: ${err.message}`;
      setStatus("error", "could not load");
      return;
    }
    await loadThumbnails();
    await renderOrderFigures().catch(() => {});
  }

  init();
})();
