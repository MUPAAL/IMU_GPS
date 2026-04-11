const wsUrl = `ws://${location.hostname}:${(Number(location.port || 8895) + 1)}`;
const el = (id) => document.getElementById(id);

function setv(id, v) { el(id).textContent = v; }

const ws = new WebSocket(wsUrl);
ws.onopen = () => setv("ws", `connected ${wsUrl}`);
ws.onclose = () => setv("ws", "disconnected");
ws.onerror = () => setv("ws", "error");
ws.onmessage = (ev) => {
  const d = JSON.parse(ev.data);
  const o = d.path_observation || {};
  const c = d.cmd_vel || {};
  setv("cxn", o.center_x_norm ?? "-");
  setv("hed", o.heading_err_deg ?? "-");
  setv("conf", o.confidence ?? "-");
  setv("lin", c.linear ?? "-");
  setv("ang", c.angular ?? "-");
  setv("rea", c.reason ?? "-");
  setv("send", String(d.enable_send_commands));
};
