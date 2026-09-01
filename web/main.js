// The loader for the wasm bundle: probe WebGPU, start the app on the
// full-window canvas, and surface a Rust panic as a page rather than a line
// in a console nobody opened (docs/plans/web-demo.md step 1).
//
// WebGPU is the only backend (ADR-0017): the viewport's pick pass reads an
// R32Uint target back with `copy_texture_to_buffer`, which wgpu's GL backend
// will not do. A browser without it gets the message below and nothing else.

import init, { WebHandle } from "./riggen_app.js";

const canvas = document.getElementById("riggen_canvas");
const message = document.getElementById("message");

/** Replace the overlay with a heading, some paragraphs and optional detail. */
function sheet(title, paragraphs, detail) {
  const box = document.createElement("div");
  box.className = "sheet";
  const h = document.createElement("h1");
  h.textContent = title;
  box.append(h);
  for (const text of paragraphs) {
    const p = document.createElement("p");
    // `innerHTML` is never used here: every string below is ours, but the
    // panic detail is not, and one code path for both is one fewer mistake.
    p.textContent = text;
    box.append(p);
  }
  if (detail) {
    const pre = document.createElement("pre");
    pre.textContent = detail;
    box.append(pre);
  }
  message.replaceChildren(box);
  message.hidden = false;
  canvas.hidden = true;
}

/**
 * True when this browser can actually give wgpu a WebGPU adapter.
 * `navigator.gpu` alone is not enough — Linux Chrome exposes it and then
 * hands back `null` when no Vulkan driver is behind it.
 */
async function hasWebGpu() {
  if (!navigator.gpu) {
    return false;
  }
  try {
    return (await navigator.gpu.requestAdapter()) !== null;
  } catch {
    return false;
  }
}

const NO_WEBGPU = [
  "riggen draws through WebGPU, and this browser does not offer it. " +
    "Everything else on the page is fine — there is just no GPU to talk to.",
  "Chrome or Edge 113+, Firefox 141+, or Safari 26+ on a machine with a " +
    "working graphics driver will run it. On Linux, Chrome may also need " +
    "chrome://flags/#enable-unsafe-webgpu.",
  "The desktop app has no such requirement: pip install riggen.",
];

async function main() {
  if (!(await hasWebGpu())) {
    sheet("This browser has no WebGPU", NO_WEBGPU);
    return;
  }

  let handle;
  try {
    await init();
    handle = new WebHandle();
    await handle.start(canvas);
  } catch (error) {
    // A panic inside `start` arrives here as a thrown value; the panic hook
    // has already recorded the good message, so prefer it.
    const summary = handle?.panic_message();
    sheet(
      "riggen failed to start",
      [summary ?? "The wasm bundle did not come up."],
      handle?.panic_callstack() ?? String(error),
    );
    return;
  }

  message.hidden = true;

  // A panic after startup poisons the runner and the canvas simply stops
  // repainting, which looks like a hang. Poll for it and say so instead.
  const poll = setInterval(() => {
    if (!handle.has_panicked()) {
      return;
    }
    clearInterval(poll);
    sheet(
      "riggen has crashed",
      [
        handle.panic_message() ?? "The app panicked.",
        "Reload the page to start over. If it repeats, the callstack below " +
          "belongs in a bug report.",
      ],
      handle.panic_callstack(),
    );
  }, 1000);
}

main();
