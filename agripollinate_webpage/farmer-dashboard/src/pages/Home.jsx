import Navbar from "../components/NavBar";
import Sidebar from "../components/SideBar";
import PollinationActivity from "../components/PollinationActivity";
import PestAlerts from "../components/PestAlerts";
import TopPollinators from "../components/TopPollinators";
import InsectSpeciesCount from "../components/InsectSpeciesCount";
import ActionableInsights from "../components/ActionableInsights";
import ActivityTrends from "../components/ActivityTrends";

export default function Home() {
  return (
    <div className="bg-slate-100 min-h-screen flex flex-col">
      <Navbar />
      <div className="flex flex-1">
        <Sidebar />
        <main className="flex-1 px-8 py-7 grid grid-cols-3 gap-6 grid-rows-[auto,auto,auto]">
          {/* Top Row */}
          <PollinationActivity />
          <PestAlerts />

          {/* Middle Row */}
          <InsectSpeciesCount />
          <TopPollinators />
          
          {/* Lower Row */}
          <ActionableInsights />
          <ActivityTrends />
        </main>
      </div>
    </div>
  );
}