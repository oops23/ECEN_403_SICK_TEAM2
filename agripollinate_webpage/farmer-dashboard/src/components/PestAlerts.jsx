import Card from "./Card";
export default function PestAlerts() {
  return (
    <Card className="row-span-1 flex flex-col gap-3 bg-white border-2 border-blue-200 rounded-lg shadow-lg p-6">
      <div className="font-semibold text-slate-700">Pest Alerts</div>
      <div className="bg-red-100 text-red-700 rounded-md p-3 flex flex-col gap-1">
        <span className="font-medium">Aphid Infestation</span>
        <span className="text-sm">Detected – South East Zone</span>
      </div>
    </Card>
  );
}