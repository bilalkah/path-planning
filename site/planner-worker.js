/*
 * Runs the C++ planners, compiled to WebAssembly (web/planner_bindings.cpp),
 * off the page's main thread.
 *
 * Messages in:
 *   { id, type: "loadMap", file }                  fetch maps/<file>, read it with planning::Map
 *   { id, type: "plan", file, planner, start, goal, params }
 * Messages out:
 *   { id, ready: true } once the module has loaded
 *   { id, result } or { id, error }
 */
/* global createPathPlanning */
"use strict";

importScripts("wasm/path_planning.js");

// Lines the C++ code writes to std::cout during a run.
let stdout = [];

const modulePromise = createPathPlanning({
  locateFile: (file) => "wasm/" + file,
  print: (line) => stdout.push(line),
  printErr: (line) => stdout.push(line),
});

modulePromise.then(
  () => self.postMessage({ id: 0, ready: true }),
  (err) => self.postMessage({ id: 0, error: "Could not load the WebAssembly module: " + err })
);

async function loadMap(module, file) {
  const response = await fetch("maps/" + file);
  if (!response.ok) throw new Error("Could not fetch maps/" + file);
  const map = module.loadMap("/" + file, await response.text());
  if (map.error) throw new Error(map.error);
  return map;
}

function plan(module, msg) {
  stdout = [];
  const result = module.plan(
    "/" + msg.file,
    msg.planner,
    msg.start.x,
    msg.start.y,
    msg.goal.x,
    msg.goal.y,
    msg.params
  );
  if (result.error) throw new Error(result.error);
  result.stdout = stdout;
  stdout = [];
  return result;
}

self.onmessage = async (event) => {
  const msg = event.data;
  try {
    const module = await modulePromise;
    if (msg.type === "loadMap") {
      const map = await loadMap(module, msg.file);
      self.postMessage({ id: msg.id, result: map }, [map.cells.buffer]);
    } else if (msg.type === "plan") {
      const result = plan(module, msg);
      self.postMessage({ id: msg.id, result }, [result.path.buffer, result.log.buffer]);
    }
  } catch (err) {
    self.postMessage({ id: msg.id, error: String((err && err.message) || err) });
  }
};
