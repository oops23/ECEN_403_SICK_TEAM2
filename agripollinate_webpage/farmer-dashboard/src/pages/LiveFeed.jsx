import React, { useEffect, useState, useRef } from "react";
import Navbar from "../components/NavBar";
import Sidebar from "../components/SideBar";
import insectStore from "../state/insectStore";      // Shared state for pollinator/pest counts
import activityStore from "../state/activityStore";  // Shared activity log (used for activity trends chart)
// TensorFlow.js
import * as tf from "@tensorflow/tfjs";

// List of all possible species predicted by model (must match your model’s order!)
const SPECIES_LIST = ["Bee", "Beetle", "Butterfly", "Dragonfly", "Grasshopper", "Ladybug", "Spider", "Wasp"];

/**
 * Parse the "hour" from filenames like "captured_10-28-2025_13.53.53.709.jpg"
 * Assumes format with '_' separating date/time from file base.
 * Returns integer hour (or null if invalid).
 */
function getHourFromFilename(filename) {
  if (!filename || typeof filename !== 'string') return null;
  const parts = filename.split('_');
  if (parts.length < 2) return null;
  const timePart = parts[parts.length - 1];    // e.g., "13.53.53.709.jpg"
  const hour = timePart.split('.')[0];         // e.g., "13"
  return isNaN(hour) ? null : parseInt(hour, 10);
}

/**
 * After a burst of images is classified, update the global counts based on their predictions.
 * Assumes model outputs for burstPreds: [{[0]=speciesPred, [1]=rolePred}, ...]
 * Only increments counts if all 5 images are unanimously classified role-wise.
 */
function processBurstClassifications(burstPreds) {
  // For each image, get the predicted species and role labels
  const topSpecies = burstPreds.map(pair => pair?.[0]?.className || null);
  const topRoles   = burstPreds.map(pair => pair?.[1]?.className || null);

  // Use the role predicted for the *first* image in the burst (you could majority-vote for robustness)
  const firstRole = topRoles[0];
  const allSameRole = topRoles.every(role => role && role.toLowerCase() === firstRole.toLowerCase());

  if (!allSameRole) return;  // Only increment count if all roles agree

  const targetSpecies = topSpecies[0];
  const roleLabel = firstRole; // "Pollinator" or "Pest" per your model

  // Update whichever store the classifier says
  if (roleLabel === "Pollinator") {
    insectStore.update('pollinators', [{ label: targetSpecies, value: 1 }]);
  } else if (roleLabel === "Pest") {
    insectStore.update('pests', [{ label: targetSpecies, value: 1 }]);
  }
  // If your model can give other roles, you may want an "else" here.
}

export default function LiveFeed() {
  // State: list of available images, modal select, modal open state
  const [images, setImages] = useState([]);
  const [selected, setSelected] = useState(null);   // filename of selected image
  const [modalOpen, setModalOpen] = useState(false);

  // State: loaded Tensorflow model, current classification result, bounding box
  const [model, setModel] = useState(null);
  const [preds, setPreds] = useState([]);           // For display in modal for selected image
  const [boxCoords, setBoxCoords] = useState(null);

  // Refs for DOM access to image and overlay during classification
  const imgRef = useRef(null);
  const boxRef = useRef(null);

  // Burst tracking: for batch classification results
  const [burstPreds, setBurstPreds] = useState([]);           // Each: [{species, role}, ...]
  const [burstFilenames, setBurstFilenames] = useState([]);   // Parallel array: image filenames

  // Load your custom Keras model ONCE on mount.
  useEffect(() => {
    async function loadModel() {
      const m = await tf.loadLayersModel('/models/tfjs_model2/model.json');
      setModel(m);
    }
    loadModel();
  }, []);

  // Fetch images from the backend API, auto-refresh every 5s for updates.
  useEffect(() => {
    async function fetchImages() {
      try {
        const res = await fetch("http://localhost:3000/images/list"); // Change URL if needed
        const files = await res.json();
        setImages(files);
      } catch (err) {
        console.error("Failed to fetch images", err);
      }
    }
    fetchImages();                               // Fetch on mount
    const interval = setInterval(fetchImages, 5000); // Repeat every 5 sec
    return () => clearInterval(interval);        // Cleanup on unmount
  }, []);

  // When a modal is opened, and a model is loaded, classify the selected image
  useEffect(() => {
    if (!modalOpen || !selected || !model) {
      setPreds([]);
      setBoxCoords(null);                        // Clear state on close
      return;
    }
    // The full prediction/classification and bounding box logic:
    async function classifyAndBox() {
      const imgEl = imgRef.current;
      if (imgEl && imgEl.complete && imgEl.naturalWidth) {
        // (Optional) Compute bounding box coordinates (~centered rectangle)
        const perc = 0.6, w = imgEl.width, h = imgEl.height;
        const bw = Math.round(w * perc), bh = Math.round(h * perc);
        const left = Math.round((w - bw) / 2), top = Math.round((h - bh) / 2);
        const boxEl = boxRef.current;
        if (boxEl) {
          boxEl.style.display = "block";
          boxEl.style.position = "absolute";
          boxEl.style.left   = left + "px";
          boxEl.style.top    = top + "px";
          boxEl.style.width  = bw  + "px";
          boxEl.style.height = bh  + "px";
          boxEl.style.border = "2px solid #ffa500";
          boxEl.style.pointerEvents = "none";
        }
        setBoxCoords({ left, top, bw, bh });

        // --- Tensor preparation and forward pass (preprocessing image) ---
        let imgTensor = tf.browser.fromPixels(imgEl)
          .resizeBilinear([224, 224])
          .div(255, 0)             // [0,1] normalization (required by your model)
          .expandDims(0);          // Add batch dimension: [1, 224, 224, 3]

        // Model outputs: [speciesTensor, roleTensor]
        const result = await model.predict(imgTensor);

        // Parse species output
        const speciesTensor = result[0];                     // Shape: [1, numSpecies]
        const roleTensor    = result[1];                     // Shape: [1, 1] (probability it's a pollinator)
        const speciesIdx  = speciesTensor.argMax(-1).dataSync()[0];
        const speciesProb = speciesTensor.max(-1).dataSync()[0];
        const speciesLabel = SPECIES_LIST[speciesIdx] || 'Unknown';

        // Parse role output; threshold for pollinator/pest role
        const roleProb = roleTensor.dataSync()[0];           // [0,1], higher is pollinator
        const pollinatorProb = roleProb;
        const pestProb = 1 - roleProb;
        const roleLabel = pollinatorProb > 0.5 ? 'Pollinator' : 'Pest';
        const roleConfidence = roleLabel === 'Pollinator' ? pollinatorProb : pestProb;

        // Format result for user-friendly modal/readout
        setPreds([
          { className: speciesLabel, probability: speciesProb },
          { className: roleLabel, probability: roleConfidence },
        ]);

        // Cleanup tensors
        imgTensor.dispose();
        speciesTensor.dispose();
        roleTensor.dispose();
      }
    }
    // Ensure the image is loaded before classifying; call classification as appropriate
    const imgEl = imgRef.current;
    if (imgEl && imgEl.complete && imgEl.naturalWidth) {
      classifyAndBox();
    } else if (imgEl) {
      imgEl.onload = classifyAndBox;
    }
  }, [modalOpen, selected, model]);

  // === BURST LOGIC ===
  // When modal closes (modalOpen goes from true to false), add the model's preds to the burst queue
  useEffect(() => {
    if (!modalOpen && preds.length && selected) {
      setBurstPreds(prev => {
        const nextPreds = [...prev, preds];
        setBurstFilenames(prevFiles => {
          const nextFiles = [...prevFiles, selected];
          // Once we've got 5 image-pred pairs in the burst...
          if (nextPreds.length >= 5 && nextFiles.length >= 5) {
            // For trends: only add one activity record per burst!
            const hour = getHourFromFilename(nextFiles[0]);
            const species = nextPreds[0]?.[0]?.className || "Unknown";
            activityStore.addActivity([{ hour, species }]);

            // For graph/pie breakdown: update insect counts via processBurstClassifications
            processBurstClassifications(nextPreds, nextFiles);
            setBurstFilenames([]);
            return []; // Reset burst
          }
          return nextFiles;
        });
        if (nextPreds.length >= 5) {
          return []; // Reset burst
        }
        return nextPreds;
      });
    }
  }, [modalOpen]);

  // --- RENDER (JSX) ---
  return (
    <div className="min-h-screen bg-slate-100">
      <Navbar />
      <div className="flex">
        <Sidebar />
        <main className="flex-1 p-6">
          <div className="flex items-center justify-between">
            <h1 className="text-lg font-semibold text-slate-700">Live Feed</h1>
            <div className="text-sm text-slate-400">Updated just now</div>
          </div>
          <div className="mt-4 bg-white rounded-lg border border-slate-100 h-[540px] flex flex-wrap items-center justify-center text-slate-400 overflow-auto">
            {/* Display all received images; clicking pops up the modal + runs classifier */}
            {images.length === 0 ? (
              <div>No images yet</div>
            ) : (
              images.map(filename => (
                <img
                  key={filename}
                  src={`http://localhost:3000/images/${filename}`}
                  alt={filename}
                  className="h-64 m-2 rounded shadow cursor-pointer"
                  style={{ objectFit: 'cover', position: 'relative' }}
                  onClick={() => {
                    setSelected(filename);
                    setModalOpen(true);
                  }}
                />
              ))
            )}
          </div>
        </main>
      </div>
      {/* === Modal: Show image, predictions, and bounding box when active === */}
      {modalOpen && selected && (
        <div
          style={{
            position: "fixed",
            zIndex: 50,
            inset: 0,
            background: "rgba(0,0,0,0.6)"
          }}
          onClick={() => setModalOpen(false)}
        >
          <div
            style={{
              position: "absolute",
              top: "10%",
              left: "50%",
              transform: "translateX(-50%)",
              background: "white",
              borderRadius: "1rem",
              boxShadow: "0 8px 24px rgba(0,0,0,0.18)",
              padding: "2rem",
              minWidth: "480px"
            }}
            onClick={e => e.stopPropagation()}
          >
            <div style={{ position: "relative", display: "inline-block" }}>
              <img
                ref={imgRef}
                src={`http://localhost:3000/images/${selected}`}
                alt={selected}
                crossOrigin="anonymous"
                style={{ width: 400, height: 'auto', borderRadius: 10 }}
              />
              {/* Bounding box overlay */}
              <div
                ref={boxRef}
                style={{
                  display: "none",
                  border: "2px solid #ff4400",
                  borderRadius: "8px",
                  position: "absolute"
                }}
              />
            </div>
            <div style={{ marginTop: "1.25rem", minHeight: "4rem" }}>
              {/* Results (species and role probabilities) */}
              {boxCoords && (
                <div style={{ color: "#222", fontWeight: "500" }}>
                  <strong>Bounding box:</strong>{" "}
                  x={boxCoords.left} y={boxCoords.top} w={boxCoords.bw} h={boxCoords.bh}
                </div>
              )}
              <ol style={{ color: "#222", fontWeight: "500" }}>
                {preds.map((p, i) => (
                  <li key={p.className}>
                    {i === 0 
                      ? `Species: ${p.className} — ${(p.probability * 100).toFixed(2)}%`
                      : `Role: ${p.className} — ${(p.probability * 100).toFixed(2)}%`
                    }
                  </li>
                ))}
              </ol>
            </div>
            <button
              className="mt-4 px-4 py-2 bg-blue-600 text-white rounded"
              onClick={() => setModalOpen(false)}
            >Close</button>
          </div>
        </div>
      )}
    </div>
  );
}