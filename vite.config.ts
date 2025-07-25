import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [react()],
  optimizeDeps: {
    exclude: ['lucide-react'],
  },
  server: {
    host: true, // Allow external connections
    port: 5174,
    allowedHosts: [
      'localhost',
      '127.0.0.1',
      '.ngrok-free.app', // Allow all ngrok subdomains
      '.ngrok.io', // For older ngrok domains
    ],
    // Alternative: disable host check entirely (less secure)
    // allowedHosts: 'all'
  },
  preview: {
    host: true,
    port: 4173,
    allowedHosts: [
      'localhost',
      '127.0.0.1',
      '.ngrok-free.app',
      '.ngrok.io',
    ],
  }
});
