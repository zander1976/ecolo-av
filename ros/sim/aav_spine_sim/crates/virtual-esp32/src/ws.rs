//! The WebSocket's few parts the status stream needs (RFC 6455): the handshake's accept
//! key, and unmasked text frames from the server. SHA-1 and base64 are written out here
//! (FIPS 180-4, RFC 4648), as `esp32_page.py` uses Python's, so the node needs no crate
//! for them. SHA-1 is only the handshake's checksum here, never for security.

/// RFC 6455's GUID, appended to the client's key.
const GUID: &str = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

/// The `Sec-WebSocket-Accept` for a client's `Sec-WebSocket-Key` (RFC 6455, 4.2.2).
#[must_use]
pub fn accept_key(key: &str) -> String {
    let mut text = String::with_capacity(key.len() + GUID.len());
    text.push_str(key.trim());
    text.push_str(GUID);
    base64(&sha1(text.as_bytes()))
}

/// One unmasked text frame, server to client.
#[must_use]
pub fn text_frame(text: &str) -> Vec<u8> {
    let data = text.as_bytes();
    let mut out = Vec::with_capacity(data.len() + 10);
    out.push(0x81);
    match data.len() {
        n if n < 126 => out.push(n as u8),
        n if n < 65_536 => {
            out.push(126);
            out.extend_from_slice(&(n as u16).to_be_bytes());
        }
        n => {
            out.push(127);
            out.extend_from_slice(&(n as u64).to_be_bytes());
        }
    }
    out.extend_from_slice(data);
    out
}

/// SHA-1 of `data` (FIPS 180-4, 6.1).
#[must_use]
pub fn sha1(data: &[u8]) -> [u8; 20] {
    let mut h: [u32; 5] = [
        0x6745_2301,
        0xEFCD_AB89,
        0x98BA_DCFE,
        0x1032_5476,
        0xC3D2_E1F0,
    ];
    let mut message = data.to_vec();
    let bits = (data.len() as u64).wrapping_mul(8);
    message.push(0x80);
    while message.len() % 64 != 56 {
        message.push(0);
    }
    message.extend_from_slice(&bits.to_be_bytes());
    let (blocks, _) = message.as_chunks::<64>();
    for block in blocks {
        let mut w = [0u32; 80];
        let (words, _) = block.as_chunks::<4>();
        for (word, bytes) in w.iter_mut().zip(words) {
            *word = u32::from_be_bytes(*bytes);
        }
        for i in 16..80 {
            w[i] = (w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16]).rotate_left(1);
        }
        let [mut a, mut b, mut c, mut d, mut e] = h;
        for (i, &word) in w.iter().enumerate() {
            let (f, k) = match i {
                0..=19 => ((b & c) | (!b & d), 0x5A82_7999),
                20..=39 => (b ^ c ^ d, 0x6ED9_EBA1),
                40..=59 => ((b & c) | (b & d) | (c & d), 0x8F1B_BCDC),
                _ => (b ^ c ^ d, 0xCA62_C1D6),
            };
            let t = a
                .rotate_left(5)
                .wrapping_add(f)
                .wrapping_add(e)
                .wrapping_add(k)
                .wrapping_add(word);
            e = d;
            d = c;
            c = b.rotate_left(30);
            b = a;
            a = t;
        }
        for (x, y) in h.iter_mut().zip([a, b, c, d, e]) {
            *x = x.wrapping_add(y);
        }
    }
    let mut out = [0u8; 20];
    let (slots, _) = out.as_chunks_mut::<4>();
    for (bytes, word) in slots.iter_mut().zip(h) {
        *bytes = word.to_be_bytes();
    }
    out
}

/// Standard base64 with padding (RFC 4648, 4).
#[must_use]
pub fn base64(data: &[u8]) -> String {
    const ABC: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(data.len().div_ceil(3) * 4);
    for chunk in data.chunks(3) {
        let b = [
            chunk[0],
            chunk.get(1).copied().unwrap_or(0),
            chunk.get(2).copied().unwrap_or(0),
        ];
        let n = u32::from(b[0]) << 16 | u32::from(b[1]) << 8 | u32::from(b[2]);
        for i in 0..4 {
            if i <= chunk.len() {
                out.push(char::from(ABC[(n >> (18 - 6 * i) & 0x3F) as usize]));
            } else {
                out.push('=');
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_rfc_example_key() {
        // RFC 6455, 1.3.
        assert_eq!(
            accept_key("dGhlIHNhbXBsZSBub25jZQ=="),
            "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="
        );
    }

    #[test]
    fn sha1_and_base64_known_answers() {
        let hex: String = sha1(b"abc").iter().map(|b| format!("{b:02x}")).collect();
        assert_eq!(hex, "a9993e364706816aba3e25717850c26c9cd0d89d");
        assert_eq!(base64(b"f"), "Zg==");
        assert_eq!(base64(b"fo"), "Zm8=");
        assert_eq!(base64(b"foo"), "Zm9v");
    }

    #[test]
    fn frames_carry_their_length() {
        assert_eq!(text_frame("hi"), [0x81, 2, b'h', b'i']);
        let long = "x".repeat(300);
        let f = text_frame(&long);
        assert_eq!(&f[..4], &[0x81, 126, 1, 44]);
        assert_eq!(f.len(), 304);
    }
}
