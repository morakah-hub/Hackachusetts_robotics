import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// Production: the backend serves frontend/dist on the same origin (start.ps1).
// Development (`npm run dev`): Vite proxies /api (HTTP + WebSocket) to the local backend.
export default defineConfig({
  plugins: [react()],
  server: {
    host: "127.0.0.1",
    port: 5173,
    proxy: {
      "/api": { target: "http://127.0.0.1:8765", ws: true, changeOrigin: false },
    },
  },
  build: { outDir: "dist", chunkSizeWarningLimit: 1500 },
});
