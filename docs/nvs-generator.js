/**
 * NVS Partition Generator for ESP32
 *
 * Generates NVS (Non-Volatile Storage) partition binaries that can be flashed
 * to ESP32 devices. This is a client-side JavaScript implementation of the
 * ESP-IDF nvs_partition_gen.py functionality.
 *
 * NVS Format:
 * - Partition divided into pages (4096 bytes each)
 * - Each page contains entries (32 bytes each)
 * - First page contains partition state
 * - Entries store key-value pairs with type information
 */

class NVSGenerator {
    constructor() {
        this.PAGE_SIZE = 4096;
        this.ENTRY_SIZE = 32;
        this.ENTRIES_PER_PAGE = 126; // (4096 - 32) / 32

        // NVS entry types
        this.TYPE_U8 = 0x01;
        this.TYPE_I8 = 0x11;
        this.TYPE_U16 = 0x02;
        this.TYPE_I16 = 0x12;
        this.TYPE_U32 = 0x04;
        this.TYPE_I32 = 0x14;
        this.TYPE_STR = 0x21;
        this.TYPE_BLOB = 0x41;

        // Page states
        this.PAGE_STATE_ACTIVE = 0xFFFFFFFE;
        this.PAGE_STATE_FULL = 0xFFFFFFFC;
        this.PAGE_STATE_EMPTY = 0xFFFFFFFF;
    }

    /**
     * Generate NVS partition binary from key-value pairs
     * @param {Object} data - Key-value pairs organized by namespace
     * @param {number} partitionSize - Size of partition in bytes (default: 0x6000 = 24KB)
     * @returns {Uint8Array} - Binary data ready to flash
     */
    generate(data, partitionSize = 0x6000) {
        const numPages = Math.floor(partitionSize / this.PAGE_SIZE);
        const binary = new Uint8Array(partitionSize);
        binary.fill(0xFF); // Initialize with 0xFF (erased flash state)

        let pageIndex = 0;
        let entryIndex = 0;

        // Process each namespace
        for (const [namespace, entries] of Object.entries(data)) {
            // Add namespace entry if we have data
            if (Object.keys(entries).length > 0) {
                this.writeEntry(binary, pageIndex, entryIndex++, {
                    namespace: 0, // Namespace entry itself uses namespace 0
                    type: 0x00, // Namespace type
                    span: 1,
                    key: namespace,
                    data: new Uint8Array([namespace.length])
                });

                // Add key-value entries
                for (const [key, value] of Object.entries(entries)) {
                    const entry = this.createEntry(namespace, key, value);
                    this.writeEntry(binary, pageIndex, entryIndex++, entry);

                    // Move to next page if current page is full
                    if (entryIndex >= this.ENTRIES_PER_PAGE) {
                        this.finalizePage(binary, pageIndex, entryIndex);
                        pageIndex++;
                        entryIndex = 0;

                        if (pageIndex >= numPages) {
                            throw new Error('NVS partition size too small for data');
                        }
                    }
                }
            }
        }

        // Finalize last page
        if (entryIndex > 0) {
            this.finalizePage(binary, pageIndex, entryIndex);
        }

        return binary;
    }

    /**
     * Create an NVS entry from a key-value pair
     */
    createEntry(namespace, key, value) {
        let type, data;

        if (typeof value === 'string') {
            type = this.TYPE_STR;
            const encoder = new TextEncoder();
            const strBytes = encoder.encode(value);
            data = new Uint8Array(strBytes.length + 1); // +1 for null terminator
            data.set(strBytes);
            data[strBytes.length] = 0; // Null terminator
        } else if (typeof value === 'number') {
            if (Number.isInteger(value)) {
                if (value >= 0 && value <= 255) {
                    type = this.TYPE_U8;
                    data = new Uint8Array([value]);
                } else if (value >= 0 && value <= 65535) {
                    type = this.TYPE_U16;
                    data = new Uint8Array(2);
                    new DataView(data.buffer).setUint16(0, value, true); // little-endian
                } else {
                    type = this.TYPE_U32;
                    data = new Uint8Array(4);
                    new DataView(data.buffer).setUint32(0, value, true); // little-endian
                }
            } else {
                throw new Error('Float values not supported yet');
            }
        } else {
            throw new Error(`Unsupported value type for key ${key}: ${typeof value}`);
        }

        return {
            namespace: this.hashNamespace(namespace),
            type: type,
            span: Math.ceil(data.length / this.ENTRY_SIZE),
            key: key,
            data: data
        };
    }

    /**
     * Write an entry to the binary at the specified page and entry index
     */
    writeEntry(binary, pageIndex, entryIndex, entry) {
        const offset = pageIndex * this.PAGE_SIZE + 32 + entryIndex * this.ENTRY_SIZE;
        const view = new DataView(binary.buffer);

        // Entry format (32 bytes):
        // [0] Namespace (1 byte)
        // [1] Type (1 byte)
        // [2] Span (1 byte)
        // [3] Reserved (1 byte)
        // [4-19] Key (16 bytes, null-padded)
        // [20-23] Data length for strings/blobs (4 bytes)
        // [24-27] CRC32 (4 bytes)
        // [28-31] Reserved/Data (4 bytes, or start of data for small values)

        binary[offset + 0] = entry.namespace;
        binary[offset + 1] = entry.type;
        binary[offset + 2] = entry.span;
        binary[offset + 3] = 0xFF; // Reserved

        // Write key (max 15 chars + null terminator)
        const keyBytes = new TextEncoder().encode(entry.key.substring(0, 15));
        binary.set(keyBytes, offset + 4);
        for (let i = keyBytes.length; i < 16; i++) {
            binary[offset + 4 + i] = 0;
        }

        // For strings/blobs, write data length
        if (entry.type === this.TYPE_STR || entry.type === this.TYPE_BLOB) {
            view.setUint32(offset + 20, entry.data.length, true);
        }

        // Write data inline if it fits (≤8 bytes), otherwise in subsequent entries
        if (entry.data.length <= 8) {
            binary.set(entry.data, offset + 24);
        } else {
            // Multi-span entry - data continues in next entries
            binary.set(entry.data.slice(0, 8), offset + 24);
            let dataOffset = 8;
            for (let i = 1; i < entry.span; i++) {
                const nextEntryOffset = offset + i * this.ENTRY_SIZE;
                const chunk = entry.data.slice(dataOffset, dataOffset + this.ENTRY_SIZE);
                binary.set(chunk, nextEntryOffset);
                dataOffset += this.ENTRY_SIZE;
            }
        }

        // Calculate and write CRC32 (simplified - using checksum for now)
        const crc = this.calculateCRC32(binary.slice(offset, offset + 28));
        view.setUint32(offset + 24, crc, true);
    }

    /**
     * Finalize a page by writing the page header
     */
    finalizePage(binary, pageIndex, numEntries) {
        const offset = pageIndex * this.PAGE_SIZE;
        const view = new DataView(binary.buffer);

        // Page header (32 bytes):
        // [0-3] Page state (4 bytes)
        // [4-7] Sequence number (4 bytes)
        // [8-11] Version (4 bytes) - set to 0xFFFFFFFF (unused)
        // [12-31] Reserved

        view.setUint32(offset + 0, this.PAGE_STATE_ACTIVE, true);
        view.setUint32(offset + 4, pageIndex, true); // Sequence number
        view.setUint32(offset + 8, 0xFFFFFFFF, true); // Version (unused)

        // Calculate page CRC and store at end of header
        const headerCRC = this.calculateCRC32(binary.slice(offset, offset + 28));
        view.setUint32(offset + 28, headerCRC, true);
    }

    /**
     * Simple hash function for namespace (ESP-IDF uses CRC8)
     */
    hashNamespace(namespace) {
        let hash = 0;
        for (let i = 0; i < namespace.length; i++) {
            hash = (hash + namespace.charCodeAt(i)) & 0xFF;
        }
        return hash || 1; // Never return 0 (reserved for namespace entries)
    }

    /**
     * Calculate CRC32 checksum
     * This is a simplified implementation - ESP-IDF uses proper CRC32
     */
    calculateCRC32(data) {
        let crc = 0xFFFFFFFF;

        for (let i = 0; i < data.length; i++) {
            crc ^= data[i];
            for (let j = 0; j < 8; j++) {
                crc = (crc >>> 1) ^ (0xEDB88320 & -(crc & 1));
            }
        }

        return ~crc >>> 0; // Convert to unsigned 32-bit
    }
}

/**
 * Helper function to generate NVS partition from web form inputs
 * @param {Object} config - Configuration object from form
 * @param {string} namespace - NVS namespace (default: 'config')
 * @param {number} partitionSize - Partition size in bytes
 * @returns {Uint8Array} - NVS partition binary
 */
function generateNVSFromConfig(config, namespace = 'config', partitionSize = 0x6000) {
    const generator = new NVSGenerator();

    // Flatten config structure into namespace data
    const nvsData = {};
    nvsData[namespace] = {};

    for (const [section, fields] of Object.entries(config)) {
        for (const [field, value] of Object.entries(fields)) {
            // Create NVS key from section and field (e.g., 'wifi_ssid')
            const key = `${section}_${field}`;
            nvsData[namespace][key] = value;
        }
    }

    return generator.generate(nvsData, partitionSize);
}

// Export for use in other modules
if (typeof module !== 'undefined' && module.exports) {
    module.exports = { NVSGenerator, generateNVSFromConfig };
}
