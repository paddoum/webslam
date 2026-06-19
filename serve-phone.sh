#!/usr/bin/env bash
# Serve webslam over HTTPS on the LAN so a phone can open it (camera needs a
# secure context). Generates a self-signed cert on first run.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
PORT="${PORT:-8443}"
CERTS="$ROOT/.certs"

# Detect the Wi-Fi LAN IP (en0), fall back to the first non-loopback inet.
IP="$(ipconfig getifaddr en0 2>/dev/null || true)"
[ -z "$IP" ] && IP="$(ifconfig 2>/dev/null | awk '/inet /{print $2}' | grep -v '127.0.0.1' | head -1)"

mkdir -p "$CERTS"
if [ ! -f "$CERTS/cert.pem" ] || ! grep -q "IP:$IP" "$CERTS/cert.pem.san" 2>/dev/null; then
  echo "generating self-signed cert for IP:$IP …"
  openssl req -x509 -newkey rsa:2048 -nodes \
    -keyout "$CERTS/key.pem" -out "$CERTS/cert.pem" -days 365 \
    -subj "/CN=webslam" \
    -addext "subjectAltName=IP:$IP,DNS:localhost" >/dev/null 2>&1
  echo "IP:$IP" > "$CERTS/cert.pem.san"
fi

echo
echo "  Open on your phone (same Wi-Fi):"
echo "      https://$IP:$PORT"
echo "  You'll see a certificate warning (self-signed) — tap Advanced → Proceed."
echo "  Then tap 'switch to camera' and allow camera access."
echo
PORT="$PORT" exec node "$ROOT/serve-https.mjs"
