import React from "react";
export default function Card({ children, className = "" }) {
  return (
    <div className={`rounded-xl shadow p-5 ${className}`}>
      {children}
    </div>
  );
}