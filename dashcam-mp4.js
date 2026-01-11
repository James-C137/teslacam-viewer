/**
 * Tesla Dashcam MP4 Parser and SEI Extractor
 * Based on https://github.com/teslamotors/dashcam
 */

/**
 * DashcamMP4 - MP4 parser class for Tesla dashcam files
 */
class DashcamMP4 {
    constructor(buffer) {
        this.buffer = buffer;
        this.view = new DataView(buffer);
    }

    /**
     * Find a box by name within a range
     */
    findBox(start, end, name) {
        let offset = start;
        while (offset < end) {
            const size = this.view.getUint32(offset);
            const type = this.getString(offset + 4, 4);
            if (size === 0) break;
            if (size === 1) { offset += 16; continue; } // Extended size box
            if (type === name) return { offset, size, type };
            offset += size;
        }
        return null;
    }

    /**
     * Read ASCII string from buffer
     */
    getString(offset, length) {
        let str = '';
        for (let i = 0; i < length; i++) {
            str += String.fromCharCode(this.view.getUint8(offset + i));
        }
        return str;
    }

    /**
     * Extract timescale and frame durations from video track
     */
    getTimescaleAndDurations() {
        const moov = this.findBox(0, this.buffer.byteLength, 'moov');
        if (!moov) return null;

        const trak = this.findBox(moov.offset + 8, moov.offset + moov.size, 'trak');
        if (!trak) return null;

        const mdia = this.findBox(trak.offset + 8, trak.offset + trak.size, 'mdia');
        if (!mdia) return null;

        const mdhd = this.findBox(mdia.offset + 8, mdia.offset + mdia.size, 'mdhd');
        if (!mdhd) return null;

        const mdhdOffset = mdhd.offset + 8;
        const version = this.view.getUint8(mdhdOffset);
        const timescale = version === 0
            ? this.view.getUint32(mdhdOffset + 12)
            : this.view.getUint32(mdhdOffset + 20);

        const minf = this.findBox(mdia.offset + 8, mdia.offset + mdia.size, 'minf');
        if (!minf) return { timescale, durations: [] };

        const stbl = this.findBox(minf.offset + 8, minf.offset + minf.size, 'stbl');
        if (!stbl) return { timescale, durations: [] };

        const stts = this.findBox(stbl.offset + 8, stbl.offset + stbl.size, 'stts');
        if (!stts) return { timescale, durations: [] };

        const sttsOffset = stts.offset + 8;
        const entryCount = this.view.getUint32(sttsOffset + 4);
        const durations = [];
        let pos = sttsOffset + 8;

        for (let i = 0; i < entryCount; i++) {
            const sampleCount = this.view.getUint32(pos);
            const sampleDelta = this.view.getUint32(pos + 4);
            for (let j = 0; j < sampleCount; j++) {
                durations.push(sampleDelta);
            }
            pos += 8;
        }

        return { timescale, durations };
    }

    /**
     * Get mdat box info (contains video data)
     */
    getMdatInfo() {
        return this.findBox(0, this.buffer.byteLength, 'mdat');
    }

    /**
     * Extract SEI messages from mdat box
     * Scans for NAL type 6 (SEI) with payload type 5 (user data)
     */
    extractSeiMessages(seiType) {
        const mdat = this.getMdatInfo();
        if (!mdat) return [];

        const seiMessages = [];
        let cursor = mdat.offset + 8;
        const end = mdat.offset + mdat.size;

        while (cursor + 4 <= end) {
            const nalSize = this.view.getUint32(cursor);
            cursor += 4;

            if (nalSize < 2 || cursor + nalSize > this.view.byteLength) {
                cursor += Math.max(nalSize, 0);
                continue;
            }

            const nalType = this.view.getUint8(cursor) & 0x1F;
            const payloadType = this.view.getUint8(cursor + 1);

            // NAL type 6 = SEI, payload type 5 = user data registered ITU-T T.35
            if (nalType === 6 && payloadType === 5) {
                const decoded = this.decodeSei(
                    new Uint8Array(this.buffer.slice(cursor, cursor + nalSize)),
                    seiType
                );
                if (decoded) seiMessages.push(decoded);
            }

            cursor += nalSize;
        }

        return seiMessages;
    }

    /**
     * Decode SEI NAL unit to extract protobuf data
     */
    decodeSei(nal, seiType) {
        if (!seiType || nal.length < 4) return null;

        try {
            // Search for magic marker: repeated 0x42 bytes followed by 0x69
            let i = 3;
            while (i < nal.length && nal[i] === 0x42) i++;

            // Validate: must have found at least one 0x42 and next byte must be 0x69
            if (i <= 3 || i + 1 >= nal.length || nal[i] !== 0x69) {
                return null;
            }

            // Protobuf data starts after 0x69 marker, ends one byte before NAL end
            const protobufData = this.stripEmulationBytes(
                nal.subarray(i + 1, nal.length - 1)
            );

            if (protobufData.length < 4) return null;

            // Decode protobuf
            const message = seiType.decode(protobufData);
            return seiType.toObject(message, {
                enums: String,
                defaults: true
            });

        } catch (e) {
            return null;
        }
    }

    /**
     * Remove H.264 emulation prevention bytes
     * In H.264, 0x000003 sequences are used to prevent start code emulation
     */
    stripEmulationBytes(data) {
        const result = [];
        for (let i = 0; i < data.length; i++) {
            if (i >= 2 && data[i - 2] === 0x00 && data[i - 1] === 0x00 && data[i] === 0x03) {
                continue;
            }
            result.push(data[i]);
        }
        return new Uint8Array(result);
    }
}

/**
 * DashcamHelpers - Utility functions for protobuf and file handling
 */
const DashcamHelpers = (function() {
    let SeiMetadata = null;
    let enumFields = null;

    /**
     * Initialize protobuf by loading the .proto file
     */
    async function initProtobuf(protoPath = 'dashcam.proto') {
        if (typeof protobuf === 'undefined') {
            throw new Error('protobufjs library not loaded');
        }

        const root = await protobuf.load(protoPath);
        SeiMetadata = root.lookupType('SeiMetadata');

        // Find enum fields for value formatting
        enumFields = {};
        SeiMetadata.fieldsArray.forEach(field => {
            if (field.resolvedType && field.resolvedType.valuesById) {
                enumFields[field.name] = field.resolvedType.valuesById;
            }
        });

        return { SeiMetadata, enumFields };
    }

    /**
     * Get cached protobuf definitions
     */
    function getProtobuf() {
        return { SeiMetadata, enumFields };
    }

    /**
     * Detect camera source from filename
     */
    function detectCameraSource(filename) {
        const name = filename.toLowerCase().replace(/[-_]/g, ' ');
        const patterns = [
            { pattern: /left\s*pillar/, label: 'Left Pillar' },
            { pattern: /right\s*pillar/, label: 'Right Pillar' },
            { pattern: /left\s*repeater/, label: 'Left Repeater' },
            { pattern: /right\s*repeater/, label: 'Right Repeater' },
            { pattern: /\bleft\b/, label: 'Left' },
            { pattern: /\bright\b/, label: 'Right' },
            { pattern: /\bfront\b/, label: 'Front' },
            { pattern: /\bback\b/, label: 'Back' },
            { pattern: /\brear\b/, label: 'Rear' },
        ];
        for (const { pattern, label } of patterns) {
            if (pattern.test(name)) return label;
        }
        return null;
    }

    /**
     * Process a single video file and extract telemetry
     */
    async function processSingleVideo(file) {
        const { SeiMetadata } = getProtobuf();
        if (!SeiMetadata) {
            throw new Error('Protobuf not initialized. Call initProtobuf() first.');
        }

        const buffer = await file.arrayBuffer();
        const parser = new DashcamMP4(buffer);

        const timing = parser.getTimescaleAndDurations();
        if (!timing) throw new Error('Could not parse video timing');

        const seiMessages = parser.extractSeiMessages(SeiMetadata);
        if (seiMessages.length === 0) throw new Error('No telemetry data found');

        // Build frame timestamps
        const timestamps = [];
        let time = 0;
        for (let i = 0; i < timing.durations.length; i++) {
            timestamps.push(time / timing.timescale);
            time += timing.durations[i];
        }

        return {
            file,
            url: URL.createObjectURL(file),
            label: detectCameraSource(file.name) || 'Unknown',
            telemetry: seiMessages,
            frameTimestamps: timestamps
        };
    }

    return {
        initProtobuf,
        getProtobuf,
        detectCameraSource,
        processSingleVideo
    };
})();

// Export to window for browser use
window.DashcamMP4 = DashcamMP4;
window.DashcamHelpers = DashcamHelpers;
