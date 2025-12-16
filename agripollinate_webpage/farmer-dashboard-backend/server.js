// Built-in modules for TCP networking, file I/O, and path handling
const net = require('net');
const fs = require('fs');
const path = require('path');

// Express for HTTP API; CORS for frontend access from other ports/hosts
const express = require('express');
const cors = require('cors');
const app = express();
app.use(cors()); // Allow CORS for all routes (very permissive; suitable for dev/demo)

// Optionally: Set up Server-Sent Events (Live client notifications)
// We'll keep a list of all current SSE clients (browser event streams)
let sseClients = [];

// --- CONFIGURATION ---
const tcpPort = 12345;             // Port Raspappberry Pi/server listens on
const tcpHost = '10.250.76.217';   // Server/Pi IP address
const httpPort = 3000;             // Port for this HTTP/web server

// Directory to save images. Will create one if missing.
const downloadDir = path.join(__dirname, 'downloads');
if (!fs.existsSync(downloadDir)) fs.mkdirSync(downloadDir);

// === TCP CLIENT: Receive images from Pi/server ===

// Buffer to accumulate TCP bytes, plus finite-state machine state/variables
let buffer = Buffer.alloc(0); // Start with empty buffer
let state = 'header';         // Two possible states: 'header' (parsing metadata), 'image' (reading image data)
let curFilename = '';         // Name of current file being received
let curFileSize = 0;          // Size of current file in bytes per protocol

/**
 * Attempt to parse the custom header: format is "filename:1234"
 * Returns a parsed object or null if header incomplete.
 * - filename: string up to colon
 * - size: file size as integer
 * - headerLen: bytes consumed from buffer (so we know how to slice it off)
 */
function tryParseHeader(buf) {
    // Header: "<filename>:<filesize>", e.g. "image_01.jpg:56734"
    const colonIdx = buf.indexOf(':');                      // Look for separator
    if (colonIdx === -1) return null;                       // Not enough in buffer
    let sizeStr = '';
    let i = colonIdx + 1;
    // Collect file size digits (ASCII numerals)
    while (i < buf.length && buf[i] >= 48 && buf[i] <= 57) { // ASCII '0'-'9'
        sizeStr += String.fromCharCode(buf[i]);
        i++;
    }
    if (!sizeStr.length) return null;                       // Need more bytes
    // Return parsed result
    return {
        filename: buf.slice(0, colonIdx).toString(),
        size: parseInt(sizeStr, 10),
        headerLen: i
    };
}

/**
 * Main receive state machine. Reads headers and image bytes from buffer,
 * writes images to disk, and moves buffer pointer forward.
 */
function process(client) {
    while (true) {
        if (state === 'header') {
            // Try to parse header (filename+size)
            let header = tryParseHeader(buffer);
            if (!header) return; // Buffer incomplete—wait for more
            curFilename = header.filename;
            curFileSize = header.size;
            buffer = buffer.slice(header.headerLen); // Consume header bytes
            console.log(`Receiving file: ${curFilename} (${curFileSize} bytes)`);
            client.write('READY'); // Protocol: acknowledge so server sends file data
            state = 'image';      // Switch to image state
        }
        if (state === 'image') {
            if (buffer.length < curFileSize) return; // Buffer incomplete—wait for more
            // Got full file data; extract and save
            const imageData = buffer.slice(0, curFileSize);
            const filePath = path.join(downloadDir, curFilename);
            fs.writeFileSync(filePath, imageData);   // Write to images/ folder
            console.log(`Saved '${curFilename}' (${curFileSize} bytes)`);
            // Optional: notify connected web clients about new image with SSE/etc
            // broadcastEvent(curFilename);
            buffer = buffer.slice(curFileSize);      // Remove file data from buffer
            state = 'header';                        // Loop for next file
        }
    }
}

// --- ACTUALLY CONNECT TO THE PI/SERVER OVER TCP ---
// Create a new TCP client/socket connection
const tcpClient = new net.Socket();
tcpClient.connect(tcpPort, tcpHost, () => {
    // Once connected, log
    console.log(`Connected to TCP server at ${tcpHost}:${tcpPort}`);
});

// Upon receiving bytes, add to buffer and process as much as possible
tcpClient.on('data', (data) => {
    buffer = Buffer.concat([buffer, data]); // Concatenate new bytes
    process(tcpClient);                     // Try to extract and save files from buffer
});
tcpClient.on('end', () => {
    // If the TCP stream ends gracefully
    console.log('TCP server closed connection.');
});
tcpClient.on('close', () => {
    // If the TCP connection is closed
    console.log('TCP connection closed');
});
tcpClient.on('error', (err) => {
    // Any errors on the TCP socket
    console.error('TCP Client error:', err);
});

// === EXPRESS WEB SERVER CONFIGURATION ===

// Add CORS headers to all /images requests to make tfjs etc. work in browser
app.use('/images', (req, res, next) => {
  res.header('Access-Control-Allow-Origin', '*');
  next();
});
// Serve any image file in downloads dir under /images/filename.jpg, etc.
app.use('/images', express.static(downloadDir));

/**
 * Helper: returns the latest N jpg/jpeg images, sorted by mtime descending
 * - n: maximum number of files
 */
function getLatestImages(n = 5) {
    try {
        // List files in download dir
        const files = fs.readdirSync(downloadDir);
        // Filter only JPG/JPEG images
        const jpgs = files.filter(f => /\.(jpe?g)$/i.test(f));
        // Get file times for sorting (mtime: last modified time)
        const withTime = jpgs.map(f => ({
            file: f,
            mtime: fs.statSync(path.join(downloadDir, f)).mtime.getTime()
        }));
        // Sort newest first
        withTime.sort((a, b) => b.mtime - a.mtime);
        // Return just filenames of N newest images
        return withTime.slice(0, n).map(obj => obj.file);
    } catch (err) {
        console.error("Error reading images:", err);
        return [];
    }
}

// API endpoint: provides JSON array of latest N image filenames (for frontend poll)
app.get('/images/list', (req, res) => {
    // Optionally, could allow query param for N
    // const n = req.query.n ? parseInt(req.query.n) : 5;
    // const latest = getLatestImages(n);
    const latest = getLatestImages(5);
    res.json(latest);
});

// Start the web server!
app.listen(httpPort, () => {
    console.log(`Express server running at http://localhost:${httpPort}`);
    console.log(`Image files served from: ${downloadDir}`);
});