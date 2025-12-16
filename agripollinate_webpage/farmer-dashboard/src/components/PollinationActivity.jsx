import Card from "./Card";
export default function PollinationActivity() {
  return (
    <Card className="col-span-2 row-span-2 bg-white border-2 border-blue-200 rounded-lg shadow-lg p-6">
      <div className="font-semibold text-slate-700">Pollination Activity</div>
      <div className="bg-slate-200 h-64 w-full rounded-lg flex justify-center items-center text-gray-400">
        [Graph/Image Placeholder]
      </div>
    </Card>
  );
}