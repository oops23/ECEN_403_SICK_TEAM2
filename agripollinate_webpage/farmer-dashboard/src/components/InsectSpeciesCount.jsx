import React, { useEffect, useRef, useState } from 'react'
import Card from './Card'
import insectStore from '../state/insectStore'

/**
 * PieChart: Display a conic-gradient pie chart of species/role breakdown.
 * - data: array of { label, value, color }
 * - size: diameter in px
 */
function PieChart({ data = [], size = 140 }) {
  // Compute total for normalization (sum of all species counts)
  const total = data.reduce((s, d) => s + Number(d.value || 0), 0)
  // Special case: if no counts, show empty (gray) pie
  if (total === 0){
    return (
      <div className="flex flex-col items-center">
        <div
          style = {{width: size, height: size, borderRadius: '50%'}}
          className = "bg-slate-200 flex items-center justify-center"
        />
        <div className = "text-xs text-slate-500 mt-2">Total: 0</div>
      </div>
    )
  }
  
  // Build conic-gradient stops for each species
  let acc = 0
  const stops = data.map((d) => {
    const pct = (Number(d.value || 0) / total) * 100    // this species' portion of 360deg
    const from = acc
    acc += pct
    const to = acc
    return `${d.color || '#60A5FA'} ${from}% ${to}%`    // CSS conic syntax for that arc
  })
  const bg = `conic-gradient(${stops.join(', ')})`

  // Pie chart
  return (
    <div className="flex flex-col items-center">
      <div
        role="img"
        aria-label="pie chart"
        style={{
          width: size,
          height: size,
          borderRadius: '50%',
          background: bg,
          boxShadow: 'inset 0 -6px 12px rgba(0,0,0,0.06)', // subtle inner shadow
        }}
      />
      <div className="text-xs text-slate-500 mt-2">Total: {total}</div>
    </div>
  )
}

/**
 * Main dashboard card: Shows pie charts + breakdowns for pollinators and pests.
 * - Uses insectStore for real-time updates
 */
export default function InsectSpeciesCount() {
  // Subscribe to the global store (keeps card up-to-date whenever counts change)
  const [storeState, setStoreState] = useState(insectStore.getState())
  useEffect(() => insectStore.subscribe(s => setStoreState({ ...s })), [])

  // Split into two arrays by type
  const pollinators = storeState.pollinators || []
  const pests = storeState.pests || []

  // For percentage calculations
  const totalPollinators = pollinators.reduce((s, d) => s + Number(d.value || 0), 0)
  const totalPests = pests.reduce((s, d) => s + Number(d.value || 0), 0)

  return (
    <Card className="col-span-1 row-span-2 bg-white border-2 border-blue-200 rounded-lg shadow-lg p-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="font-semibold text-slate-700">Insect Species Count</div>
        <div className="text-sm text-slate-400">Updated just now</div>
      </div>
      {/* Pie charts for pollinators and pests */}
      <div className="mt-4 flex items-center justify-center gap-6">
        <PieChart data={pollinators} size={120} />
        <PieChart data={pests} size={120} />
      </div>
      {/* Detailed breakdown for both roles */}
      <div className="mt-6">
        <div className="grid grid-cols-2 gap-4">
          {/* Left: pollinators table */}
          <div className="flex flex-col items-start gap-2">
            <div className="text-sm font-medium text-slate-600">Pollinators</div>
            <div className="flex flex-col gap-2 w-full">
              {pollinators.map((p) => (
                <div key={p.label} className="flex items-center gap-3 justify-between">
                  <div className="flex items-center gap-2">
                    {/* Color swatch */}
                    <div style={{ width: 12, height: 12, background: p.color }} className="rounded-sm" />
                    <div className="text-sm text-slate-700">{p.label}</div>
                  </div>
                  {/* Percentage of species in this group */}
                  <div className="text-xs text-slate-500">
                    {totalPollinators ? `${((p.value / totalPollinators) * 100).toFixed(1)}%` : '0%'}
                  </div>
                </div>
              ))}
            </div>
          </div>
          {/* Right: pests table */}
          <div className="flex flex-col items-start gap-2">
            <div className="text-sm font-medium text-slate-600">Pests</div>
            <div className="flex flex-col gap-2 w-full">
              {pests.map((p) => (
                <div key={p.label} className="flex items-center gap-3 justify-between">
                  <div className="flex items-center gap-2">
                    {/* Color swatch */}
                    <div style={{ width: 12, height: 12, background: p.color }} className="rounded-sm" />
                    <div className="text-sm text-slate-700">{p.label}</div>
                  </div>
                  {/* Percentage of species in this group */}
                  <div className="text-xs text-slate-500">
                    {totalPests ? `${((p.value / totalPests) * 100).toFixed(1)}%` : '0%'}
                  </div>
                </div>
              ))}
            </div>
          </div>
        </div>
      </div>
    </Card>
  )
}