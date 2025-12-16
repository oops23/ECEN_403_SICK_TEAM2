import React from "react";
import logo from "../assets/logo.png";
export default function Navbar() {
  return (
    <header className="bg-gradient-to-r from-blue-600 to-green-600 p-4 shadow">
      <div className="flex items-center justify-between">
        {/* Left side: logo and title */}
        <div className="flex items-center gap-3">
          <img src={logo} alt="logo" className="rounded-full h-8"/>
          <span className="text-xl font-semibold text-white">AgriPollinate</span>
        </div>
        {/* Right side: last updated and user icons */}
        <div className="flex items-center gap-6">
          <div className="text-sm text-gray-200">Last Updated: Just now</div>
          <div className="flex items-center gap-2">
            <button className="rounded-full bg-gray-100 p-2"><span role="img" aria-label="User">👤</span></button>
          </div>
        </div>
      </div>
    </header>
  );
  
}