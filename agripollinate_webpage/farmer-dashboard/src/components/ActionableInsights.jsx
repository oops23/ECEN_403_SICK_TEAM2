import Card from "./Card";
export default function ActionableInsights() {
  return (
    <Card className="col-span-3 bg-white border-2 border-blue-200 rounded-lg shadow-lg p-6">
      <div className="flex items-center justify-between">
        <div className="font-semibold text-slate-700">Actionable Insights</div>
        <div className="text-sm text-slate-400">Updated just now</div>
      </div>

      <div className="mt-4 grid grid-cols-2 gap-8">
        <div>
          <div className="font-semibold text-slate-700">Hive Relocation</div>
          <div className="text-sm text-slate-500 mt-1">
            Move Hive 2 to Zone C for improved fruit set
          </div>
          <button className="mt-3 px-3 py-1 bg-blue-100 text-blue-700 rounded">
            View Plan
          </button>
        </div>

        <div>
          <div className="font-semibold text-slate-700">Pest Management</div>
          <div className="text-sm text-slate-500 mt-1">
            Apply organic nem oil to affected area
          </div>
          <button className="mt-3 px-3 py-1 bg-blue-100 text-blue-700 rounded">
            Order Supply
          </button>
        </div>
      </div>
    </Card>
  );
}