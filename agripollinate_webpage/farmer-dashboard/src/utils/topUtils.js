export function normalizeTopList(list) {
  return [...list].map(item => ({ label: String(item.label), value: Number(item.value) }))
}

export function checkAndInsertTop(topList, candidate, maxLen = 3) {
  // topList: array of {label, value} (doesn't need to be sorted)
  // candidate: {label, value}
  const normalized = normalizeTopList(topList)
  const cand = normalizeTopList([candidate])[0]

  // update or add
  const idx = normalized.findIndex(i => i.label === cand.label)
  if (idx >= 0) normalized[idx].value = cand.value
  else normalized.push(cand)

  // sort and slice top N
  normalized.sort((a, b) => b.value - a.value)
  const newTop = normalized.slice(0, maxLen)

  // compare old vs new (label+value)
  const oldTop = normalizeTopList(topList).sort((a,b)=>b.value-a.value).slice(0,maxLen)
  const changed = oldTop.length !== newTop.length ||
    oldTop.some((it, i) => it.label !== newTop[i].label || it.value !== newTop[i].value)

  // who got displaced (if any)
  const displaced = oldTop.filter(old => !newTop.some(n => n.label === old.label))

  return { changed, newTop, displaced }
}