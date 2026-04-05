// Minimal WAV reader/writer — embedded in generated simulate binaries.
// No external dependencies. Supports 16-bit int, 24-bit int, 32-bit float.

fn read_wav(path: &str) -> (Vec<f64>, f64) {
    let data = std::fs::read(path).unwrap_or_else(|e| {
        eprintln!("Failed to read WAV file '{}': {}", path, e);
        std::process::exit(1);
    });
    if data.len() < 44 || &data[0..4] != b"RIFF" || &data[8..12] != b"WAVE" {
        eprintln!("Not a valid WAV file: {}", path);
        std::process::exit(1);
    }

    // Parse fmt chunk
    let mut pos = 12;
    let mut sample_rate = 44100u32;
    let mut bits_per_sample = 16u16;
    let mut num_channels = 1u16;
    let mut audio_format = 1u16; // 1=PCM, 3=IEEE float
    let mut data_start = 0;
    let mut data_size = 0usize;

    while pos + 8 <= data.len() {
        let chunk_id = &data[pos..pos + 4];
        let chunk_size = u32::from_le_bytes([data[pos+4], data[pos+5], data[pos+6], data[pos+7]]) as usize;
        pos += 8;

        if chunk_id == b"fmt " && chunk_size >= 16 {
            audio_format = u16::from_le_bytes([data[pos], data[pos+1]]);
            num_channels = u16::from_le_bytes([data[pos+2], data[pos+3]]);
            sample_rate = u32::from_le_bytes([data[pos+4], data[pos+5], data[pos+6], data[pos+7]]);
            bits_per_sample = u16::from_le_bytes([data[pos+14], data[pos+15]]);
        } else if chunk_id == b"data" {
            data_start = pos;
            data_size = chunk_size;
            break; // Found data chunk
        }
        pos += chunk_size;
        if pos % 2 != 0 { pos += 1; } // Chunks are word-aligned
    }

    if data_start == 0 || data_size == 0 {
        eprintln!("No data chunk found in WAV file: {}", path);
        std::process::exit(1);
    }

    let bytes_per_sample = (bits_per_sample / 8) as usize;
    let frame_size = bytes_per_sample * num_channels as usize;
    let num_frames = data_size / frame_size;
    let mut samples = Vec::with_capacity(num_frames);

    for i in 0..num_frames {
        let offset = data_start + i * frame_size;
        // Read first channel only (mono downmix)
        let sample = match (audio_format, bits_per_sample) {
            (1, 16) => {
                let raw = i16::from_le_bytes([data[offset], data[offset+1]]);
                raw as f64 / 32768.0
            }
            (1, 24) => {
                let b0 = data[offset] as i32;
                let b1 = data[offset+1] as i32;
                let b2 = data[offset+2] as i32;
                let raw = (b2 << 24 | b1 << 16 | b0 << 8) >> 8; // Sign-extend
                raw as f64 / 8388608.0
            }
            (3, 32) => {
                let raw = f32::from_le_bytes([data[offset], data[offset+1], data[offset+2], data[offset+3]]);
                raw as f64
            }
            _ => {
                eprintln!("Unsupported WAV format: {} bits, format={}", bits_per_sample, audio_format);
                std::process::exit(1);
            }
        };
        samples.push(sample);
    }

    (samples, sample_rate as f64)
}

fn write_wav(path: &str, sample_rate: u32, samples: &[f64]) {
    let num_samples = samples.len();
    let data_size = (num_samples * 4) as u32; // 32-bit float
    let file_size = 36 + data_size;

    let mut buf = Vec::with_capacity(file_size as usize + 8);

    // RIFF header
    buf.extend_from_slice(b"RIFF");
    buf.extend_from_slice(&file_size.to_le_bytes());
    buf.extend_from_slice(b"WAVE");

    // fmt chunk (IEEE float)
    buf.extend_from_slice(b"fmt ");
    buf.extend_from_slice(&16u32.to_le_bytes()); // chunk size
    buf.extend_from_slice(&3u16.to_le_bytes());  // audio format (3 = IEEE float)
    buf.extend_from_slice(&1u16.to_le_bytes());  // num channels
    buf.extend_from_slice(&sample_rate.to_le_bytes());
    buf.extend_from_slice(&(sample_rate * 4).to_le_bytes()); // byte rate
    buf.extend_from_slice(&4u16.to_le_bytes());  // block align
    buf.extend_from_slice(&32u16.to_le_bytes()); // bits per sample

    // data chunk
    buf.extend_from_slice(b"data");
    buf.extend_from_slice(&data_size.to_le_bytes());
    for &s in samples {
        buf.extend_from_slice(&(s as f32).to_le_bytes());
    }

    std::fs::write(path, &buf).unwrap_or_else(|e| {
        eprintln!("Failed to write WAV file '{}': {}", path, e);
        std::process::exit(1);
    });
}
