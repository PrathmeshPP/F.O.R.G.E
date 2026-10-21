import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import path from 'path';
import fs from 'fs';

// Custom plugin to serve real reconstruction assets directly from outputs/accuracy_experiment/sprint9_video_to_viewer_release
// without copying or duplicating hundreds of megabytes of 3D models into public/.
function serveModelAssets() {
  const currentDir = import.meta.dirname || __dirname;
  const modelsDir = path.resolve(currentDir, '../outputs/accuracy_experiment/sprint9_video_to_viewer_release');

  const handler = (req: any, res: any, next: any) => {
    if (!req.url || !req.url.startsWith('/models/')) {
      return next();
    }

    const cleanUrl = req.url.split('?')[0];
    const relPath = decodeURIComponent(cleanUrl.replace(/^\/models\/?/, ''));
    const filePath = path.join(modelsDir, relPath);

    // Prevent directory traversal attacks
    if (!filePath.startsWith(modelsDir)) {
      res.statusCode = 403;
      res.end('Access denied');
      return;
    }

    if (fs.existsSync(filePath) && fs.statSync(filePath).isFile()) {
      const ext = path.extname(filePath).toLowerCase();
      const mimeTypes: Record<string, string> = {
        '.obj': 'text/plain; charset=utf-8',
        '.mtl': 'text/plain; charset=utf-8',
        '.ply': 'application/octet-stream',
        '.png': 'image/png',
        '.jpg': 'image/jpeg',
        '.jpeg': 'image/jpeg',
        '.json': 'application/json; charset=utf-8',
      };

      res.setHeader('Content-Type', mimeTypes[ext] || 'application/octet-stream');
      res.setHeader('Access-Control-Allow-Origin', '*');
      res.setHeader('Cache-Control', 'public, max-age=3600');
      fs.createReadStream(filePath).pipe(res);
      return;
    }

    res.statusCode = 404;
    res.end(`Model asset not found: ${relPath}`);
  };

  return {
    name: 'serve-model-assets',
    configureServer(server: any) {
      server.middlewares.use(handler);
    },
    configurePreviewServer(server: any) {
      server.middlewares.use(handler);
    },
  };
}

export default defineConfig({
  plugins: [react(), serveModelAssets()],
  server: {
    port: 5173,
    host: true,
    fs: {
      allow: ['..'],
    },
  },
  preview: {
    port: 5174,
    host: true,
  },
});
