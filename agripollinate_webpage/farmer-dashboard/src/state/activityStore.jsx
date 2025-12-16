let state = {
  activity: [] // Array of { hour, species }
};

const listeners = new Set();

// Get the current state (for initial value)
function getState() {
  return state;
}

// Subscribe to changes: callback gets called whenever state changes
function subscribe(cb) {
  listeners.add(cb);
  try { cb(state); } catch (e) {}
  return () => listeners.delete(cb);
}

// Add one or more new activity entries and notify listeners
function addActivity(entries) {
  // entries: [{hour, species}, ...]
  state = { ...state, activity: [...state.activity, ...entries] };
  for (const cb of Array.from(listeners)) {
    try { cb(state); } catch (e) {}
  }
}

export default {
  getState,
  subscribe,
  addActivity,
};