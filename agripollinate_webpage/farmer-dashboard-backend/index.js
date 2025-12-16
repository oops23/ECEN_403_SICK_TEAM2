// Import required built-in modules for networking, filesystem, and path manipulation
const net = require('net');
const fs = require('fs');
const path = require('path');

// TCP server configuration
const port = 12345;                 // Port number to connect to
const host = '10.250.76.217';       // Server IP address
const downloadDir = './downloads';  // Directory to save received images

// Ensure the download directory exists or create it if missing
if (!fs.existsSync(downloadDir)) fs.mkdirSync(downloadDir);

// Buffer to hold incoming data, state for protocol handling, and current image info variables
let buffer = Buffer.alloc(0); // Will accumulate incoming bytes here
let state = 'header';         // State can be 'header' (parsing filename and size) or 'image' (saving binary data)
let curFilename = '';         // Will store filename for current image being received
let curFileSize = 0;          // Will store expected file size in bytes for current image

/**
 * Attempts to parse a header (in format 'filename:size') from the beginning of the buffer.
 * Returns an object { filename, size, headerLen } if a complete header is found, else null.
 * - buf: Buffer (may contain header, partial header, and/or image data)
 */
function tryParseHeader(buf) {
    // Header format: `filename:size`, so look for the first colon
    const colonIdx = buf.indexOf(':');                        // position of the colon
    if (colonIdx === -1) return null;                         // If no colon, haven't received full header yet

    // After the colon should be a decimal ASCII string for the file size
    let sizeStr = '';
    let i = colonIdx + 1;
    while (i < buf.length && buf[i] >= 48 && buf[i] <= 57) { // ASCII 48-57 is '0'-'9'
        sizeStr += String.fromCharCode(buf[i]);
        i++;
    }
    if (!sizeStr.length) return null; // File size incomplete or missing; wait for more data

    // Return the parsed header and how many bytes the header took (so we can slice buffer correctly)
    return {
        filename: buf.slice(0, colonIdx).toString(), // From start to just before colon
        size: parseInt(sizeStr, 10),                 // Convert file size string to number
        headerLen: i                                 // Total header length (filename, colon, size digits)
    };
}

/**
 * The main state machine for receiving images.
 * Attempts to process as much of the buffer as possible (can process multiple images per 'data' event).
 * - client: The TCP connection object (net.Socket)
 */
function process(client) {
    while (true) {
        if (state === 'header') {
            // Try to extract a full header (filename and size) from the buffer
            let header = tryParseHeader(buffer);
            if (!header) return; // If incomplete, wait for more data

            // Have a full header! Store info and chop header off the buffer
            curFilename = header.filename;
            curFileSize = header.size;
            buffer = buffer.slice(header.headerLen); // Remove header bytes from buffer

            // For debugging or tracking
            console.log(`Receiving file: ${curFilename} (${curFileSize} bytes)`);

            // Our protocol: send 'READY' after parsing header so server can start sending the data
            client.write('READY');

            // Next state: expecting file data
            state = 'image';
        }
        if (state === 'image') {
            // Are there enough bytes in the buffer for the whole image?
            if (buffer.length < curFileSize) return; // Not enough yet; wait

            // We have the whole image, extract it
            const imageData = buffer.slice(0, curFileSize);

            // Where to save the image file
            const filePath = path.join(downloadDir, curFilename);

            // Write the bytes to disk
            fs.writeFileSync(filePath, imageData);
            console.log(`Saved '${curFilename}' (${curFileSize} bytes)`);

            // Remove the image bytes from the buffer (could be more headers/images remaining in buffer)
            buffer = buffer.slice(curFileSize);

            // Go back to header state for the next image in the stream, if present
            state = 'header';
        }
    }
}

// ---- Setup the TCP client and event handlers ----

// Create a new TCP socket/client
const client = new net.Socket();

// Begin connection to the given host/port on the server
client.connect(port, host, () => {
    console.log('Connected to TCP server');
});

// Listen for incoming data on the socket
client.on('data', (data) => {
    // Concatenate new data onto buffer (since a single .data event might be split or combine messages/files)
    buffer = Buffer.concat([buffer, data]);
    // Process (parse) as much of the buffer as possible (may process multiple images at once!)
    process(client);
});

// Handle TCP socket ending (server closed the connection)
client.on('end', () => {
    console.log('Server closed connection.');
});

// Handle TCP socket fully closing
client.on('close', () => {
    console.log('Connection closed');
});

// Handle TCP errors (connection and IO problems)
client.on('error', (err) => {
    console.error('Client error:', err);
});