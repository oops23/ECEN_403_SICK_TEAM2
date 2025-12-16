// simple shared store for pollinators + pests with subscribe/update/simulate
const DEFAULT_POLLINATORS = [/* ... */]

const DEFAULT_PESTS = [/* ... */]

let state = {
  pollinators: DEFAULT_POLLINATORS.slice(),
  pests: DEFAULT_PESTS.slice(),
}

const listeners = new Set()

function notify() {
  for (const cb of Array.from(listeners)) {
    try { cb(state) } catch (e) { /* ignore */ }
  }
}

function getState() {
  return state
}

function subscribe(cb) {
  listeners.add(cb)
  // call immediately with current state
  try { cb(state) } catch (e) {}
  return () => listeners.delete(cb)
}

// merge incoming items by label (replace value / color if provided)
function mergeList(oldList, incoming = []) {
  const map = Object.fromEntries(oldList.map(i => [i.label, { ...i }]))
  incoming.forEach(it => {
    if (map[it.label]) {
      // Increment value instead of replacing
      map[it.label].value = (map[it.label].value || 0) + (it.value || 1)
    } else {
      map[it.label] = { ...(it), color: it.color || makeColor() }
    }
  });
  return Object.values(map)
}

function update(type, payload = []) {
  if (type === 'pollinators') {
    state = { ...state, pollinators: mergeList(state.pollinators, payload) }
    notify()
  } else if (type === 'pests') {
    state = { ...state, pests: mergeList(state.pests, payload) }
    notify()
  }
}

// small helper to create a color for a new species (ensure uniqueness)
function makeColor() {
  const palette = ['#F59E0B','#EF4444','#10B981','#F97316','#60A5FA','#A78BFA','#F472B6','#34D399']
  // collect currently used colors from state
  const used = new Set([
    ...(state.pollinators || []).map(p => p.color),
    ...(state.pests || []).map(p => p.color),
  ])
  // pick an unused palette color if available
  const available = palette.filter(c => !used.has(c))
  if (available.length > 0) {
    return available[Math.floor(Math.random() * available.length)]
  }
  // otherwise generate a new random color not already used
  const gen = () => {
    let c
    do {
      c = '#' + Math.floor(Math.random() * 0xffffff).toString(16).padStart(6, '0')
    } while (used.has(c))
    return c
  }
  return gen()
}

// simulate: tweak existing values and sometimes add a new species
function simulate() {
  const randDelta = () => Math.round((Math.random() - 0.4) * 20)
  const maybeAdd = (list) => {
    if (Math.random() < 0.18) { // 18% chance to add a new species
      const newLabel = `Species ${Math.floor(Math.random()*900+100)}`
      list.push({ label: newLabel, value: Math.max(5, Math.round(Math.random()*40)), color: makeColor() })
    }
  }

  const newPoll = state.pollinators.map(p => ({ ...p, value: Math.max(0, p.value + randDelta()) }))
  maybeAdd(newPoll)

  const newPests = state.pests.map(p => ({ ...p, value: Math.max(0, p.value + randDelta()) }))
  maybeAdd(newPests)

  state = { ...state, pollinators: newPoll, pests: newPests }
  notify()
}

export default {
  getState,
  subscribe,
  update,
  simulate,
}