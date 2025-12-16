import React from "react";
import { NavLink } from "react-router-dom";
import { HomeIcon, CameraIcon, ChartBarIcon, QuestionMarkCircleIcon, Cog8ToothIcon } from "@heroicons/react/24/solid";

const navLinks = [
  
  { label: "Dashboard", icon: HomeIcon, path: "/" },
  { label: "Live Feed", icon: CameraIcon, path: "/live-feed" },
  { label: "Historical Data", icon: ChartBarIcon, path: "/historical-data" },
  { label: "Settings", icon: Cog8ToothIcon, path: "/settings" },
  { label: "Support", icon: QuestionMarkCircleIcon, path: "/support" },
];
export default function Sidebar() {
  return (
    <aside className="bg-slate-50 w-56 pt-8 px-5 min-h-screen flex flex-col gap-2 border-r">
      {navLinks.map((link) => {
        const Icon = link.icon;
        return (
          <NavLink
            key={link.label}
            to={link.path}
            end={link.path === '/'}
            className={({ isActive }) =>
              `flex items-center gap-3 py-2 px-3 rounded-lg text-left font-medium transition-colors ${
                isActive ? 'bg-blue-50 text-blue-700' : 'text-slate-700 hover:bg-slate-200'
              }`
            }
            aria-current={({ isActive }) => (isActive ? "page" : undefined)}
          >
            <Icon className="h-6 w-6 text-blue-500" />
            <span>{link.label}</span>
          </NavLink>
        );
      })}
    </aside>
  );
}